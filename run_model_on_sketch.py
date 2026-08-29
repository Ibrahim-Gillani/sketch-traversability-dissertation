from tkinter import filedialog
import tkinter
import rasterio
import os
from PIL import Image
from terrain_diffusion.inference.tiff_export import main as tiff_export_main
import scipy.ndimage
from tqdm import tqdm
import numpy

#Turn sketch to elevation
def image_to_elevation(image_path, min_elevation, max_elevation):
    #Open image as greyscale image
    img = Image.open(image_path).convert("L")
    #Create array
    array = numpy.array(img)
    #Remap array to desired elevations
    grid = min_elevation + ((array / 255) * (max_elevation - min_elevation))
    return grid

#Function to detect anomalies
def detect_anomalies(UEHeightmap):
    #Convert to float32
    grid_as_float = UEHeightmap.astype("float32")
    #Detect height differences horizontally and vertically
    diff_rows = numpy.abs(numpy.diff(grid_as_float, axis=0))
    diff_cols = numpy.abs(numpy.diff(grid_as_float, axis=1))
    #Pad as diff reduces row/col by 1
    padded_rows = numpy.pad(diff_rows, ((0, 1), (0, 0)), mode='edge')
    padded_cols = numpy.pad(diff_cols, ((0, 0), (0, 1)), mode='edge')
    #Overlay both to find abolsute max jump
    max_jumps = numpy.maximum(padded_rows, padded_cols)
    #Use absolute structural threshold to target real faults and skip natural terrain ridges
    cliff_mask = max_jumps > 2000.0
    return cliff_mask

#Function to fill regions
def fill_cliff_mask(cliff_mask):
    #Close single pixel diagonal leaks using a tight cross structure
    structure = scipy.ndimage.generate_binary_structure(2, 1)
    closed_mask = scipy.ndimage.binary_closing(cliff_mask, structure=structure)
    #Force the absolute outer 1-pixel boundary of the image to be a solid wall (to fix missing gaps)
    closed_mask[0, :] = True
    closed_mask[-1, :] = True
    closed_mask[:, 0] = True
    closed_mask[:, -1] = True
    #Invert the mask so largest area becomes True
    inverted = ~closed_mask
    #Label every isolated open-ground territory
    labeled_bg, num_features = scipy.ndimage.label(inverted)
    #Find the largest territory (guaranteed to be your main map terrain)
    bincounts = numpy.bincount(labeled_bg.ravel())
    bincounts[0] = 0 
    main_background_label = numpy.argmax(bincounts)
    #Create a mask of just the main terrain
    main_background_mask = (labeled_bg == main_background_label)
    #Fill aretefacts
    filled_mask = ~main_background_mask
    #Clear the forced edge wall back to False
    filled_mask[0, :] = False
    filled_mask[-1, :] = False
    filled_mask[:, 0] = False
    filled_mask[:, -1] = False
    return filled_mask

#Repair map
def correct_region_height(filled_mask, UEHeightmap):
    #Group artefact regions
    artefact_region_id, num_regions = scipy.ndimage.label(filled_mask)
    #Force grid to float32 to prevent array underflow errors
    working_grid = UEHeightmap.astype("float32")
    #Loop through regions
    for region in tqdm(range(1, num_regions + 1), desc="Repairing regions"):
        #Find region pixels
        mask_indices = (artefact_region_id == region)
        rows, cols = numpy.where(mask_indices)
        #Track all valid transitions along outer edge
        boundary_jumps = []
        neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        #Scan every pixel inside the region
        for r, c in zip(rows, cols):
            for dr, dc in neighbors:
                nr, nc = r + dr, c + dc
                #Verify neighbor coordinates sit inside the map boundary
                if 0 <= nr < artefact_region_id.shape[0] and 0 <= nc < artefact_region_id.shape[1]:
                    #Check if neighbor pixel is verified background terrain
                    if artefact_region_id[nr, nc] == 0:
                        terrain_pixel = working_grid[nr, nc]
                        artefact_pixel = working_grid[r, c]
                        boundary_jumps.append(terrain_pixel - artefact_pixel)
        #Apply the uniform offset calculation
        if boundary_jumps:
            largest_jump = max(boundary_jumps, key=abs)
            if abs(largest_jump) >= 1000.0:
                working_grid[mask_indices] += largest_jump
    #Clamp array values back to valid unsigned 16bit integers
    return numpy.clip(working_grid, 0, 65535).astype("uint16")

#Convert an elevation grid to a 16-bit unsigned PNG for Unreal
def save_as_ue_heightmap(grid, min_elev, max_elev, output_path):
    UEgrid_n = (grid - min_elev) / (max_elev - min_elev) * 65535
    UEgrid_16bit = UEgrid_n.astype("uint16")
    img = Image.fromarray(UEgrid_16bit, mode="I;16")
    img.save(output_path)
    return UEgrid_16bit
        
#RUN SCRIPT
#Input params
min_elev = float(input("Enter minimum elevation (metres): "))
max_elev = float(input("Enter maximum elevation (metres): "))

tkinter.Tk().withdraw()
image_path = filedialog.askopenfilename(title="Select your sketch image")

#Generate 2d array (Cell)
grid = image_to_elevation(
    image_path,
    min_elev,
    max_elev
)
print(grid.shape)
print(grid.min(), grid.max())

#Set up CRS and transform
#One input cell = 256 output pixels, each pixel is 30m, in the final output
MetresPerCell = 30 * 256 
cell_crs = rasterio.crs.CRS.from_epsg(3857)
cell_transform = rasterio.transform.from_origin(0, 0, MetresPerCell, MetresPerCell)

#Set up output destination
ScriptFolder = os.path.dirname(os.path.abspath(__file__))
OutputFolder = os.path.join(ScriptFolder, "SketchOutput")
os.makedirs(OutputFolder, exist_ok=True)
OutputPath = os.path.join(OutputFolder, "heightmap.tif")
GeoTIFFOutputPath = os.path.join(ScriptFolder, "GeoTIFFOutput.tif")
UEHeightmapPath = os.path.join(ScriptFolder, "UE_heightmap_raw.png")
UEHeightmapRepairedPath = os.path.join(ScriptFolder, "UE_heightmap_repaired.png")

#Open file for writing
with rasterio.open(
    OutputPath,                #the file path to create
    "w",                       #w means writing a new file, not reading one
    driver="GTiff",            #the file type is GeoTIFF
    height=grid.shape[0],      #number of rows in your grid
    width=grid.shape[1],       #number of columns in your grid
    count=1,                   #how many layers of data, only 1 for elevation
    dtype="float32",           #what kind of number each pixel is stored as
    crs=cell_crs,              #coordinate system
    transform=cell_transform,  #pixel-to-real-world mapping
) as dst:
    dst.write(grid,1)

#Call model
tiff_export_main.callback(
    tiff_dir=OutputFolder,                          #the folder containing generated heightmap.tif
    output=GeoTIFFOutputPath,                       #where to save the model's generated result
    model_path="xandergos/terrain-diffusion-30m",   #which pretrained model to use, 30m 
    snr="0.2,0.2,1.0,0.2,1.0",                      #how strongly to follow the sketch (elevation channel set to 1.0, strong)
    #Remaining tool behaviour
    hdf5_file=None,
    cache_size="1G",
    seed=None,
    device=None,
    batch_size="1,2,4,8,16",
    torch_compile=True,
    dtype="fp32",
    caching_strategy="direct",
    chunk_size=8 * 256,
)

#Convert from GeoTIFF to 16-bit unsigned PNG, after repair
#Open heightmap
with rasterio.open(GeoTIFFOutputPath) as src:
    generated_grid = src.read(1)

#Convert to UE-ready 16-bit grid once, reused for both detection and the raw save
UEgrid_16bit = save_as_ue_heightmap(generated_grid, min_elev, max_elev, UEHeightmapPath)

#Repair map from artefacts
#Run the center-seeded flood fill routine to generate clean solid mask zones
solid_mask = fill_cliff_mask(detect_anomalies(UEgrid_16bit))

#Save a visual diagnostic image of the final mask to check your regions
Image.fromarray((solid_mask > 0).astype("uint8") * 255).save(os.path.join(ScriptFolder, "detection_view.png"))

#Execute alignment correction over the solid mask regions
repaired_grid = correct_region_height(solid_mask, UEgrid_16bit)
print("Pixels changed:", numpy.sum(repaired_grid != UEgrid_16bit))

#Save final heightmap container directly without scaling errors
img = Image.fromarray(repaired_grid, mode="I;16")
img.save(UEHeightmapRepairedPath)