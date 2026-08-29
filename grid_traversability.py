import unreal
import networkx as nx
import math
import time
from shapely.geometry import Point
import shapely

#UI Functions for tool
#Function to create marker at selected POI location, spawns a large glowing red cylinder.
def spawn_marker_at_selected(label, width_multiplier = 1.0):
    EAS = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    selected = EAS.get_selected_level_actors()
    if len(selected) == 0:
        return None
    target_actor = selected[0]
    location = target_actor.get_actor_location()
    #Check if a marker with this label already exists, move it instead of spawning a duplicate
    ActorList = EAS.get_all_level_actors()
    existing = unreal.EditorFilterLibrary.by_actor_label(ActorList, label)
    if len(existing) > 0:
        existing_marker = existing[0]
        existing_marker.set_actor_location(location, False, False)
        existing_marker.set_actor_enable_collision(False)
        return existing_marker
    rotation = unreal.Rotator(0, 0, 0)
    marker = unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.StaticMeshActor, location, rotation)
    mesh = unreal.EditorAssetLibrary.load_asset("/Engine/BasicShapes/Cylinder.Cylinder")
    marker.static_mesh_component.set_static_mesh(mesh)
    marker.set_actor_scale3d(unreal.Vector(0.6 * width_multiplier, 0.6 * width_multiplier, 100.0))
    material = unreal.EditorAssetLibrary.load_asset("/Engine/EngineDebugMaterials/VertexColorViewMode_RedOnly.VertexColorViewMode_RedOnly")
    marker.static_mesh_component.set_material(0, material)
    marker.set_actor_label(label)
    marker.set_actor_enable_collision(False)
    return marker

#Fucntion to get search area bounds
def calculate_region_bounds(loc_a, loc_b, pad_left, pad_right, pad_up, pad_down):
    base_min_x = min(loc_a.x, loc_b.x)
    base_max_x = max(loc_a.x, loc_b.x)
    base_min_y = min(loc_a.y, loc_b.y)
    base_max_y = max(loc_a.y, loc_b.y)
    min_x = base_min_x - pad_left
    max_x = base_max_x + pad_right
    min_y = base_min_y - pad_down
    max_y = base_max_y + pad_up
    return min_x, max_x, min_y, max_y

#Draws a preview outline of the search region above the terrain, 
#so the designer can confirm the area before running the full traversability check
def preview_region(world, min_x, max_x, min_y, max_y, loc_a, loc_b, duration = 15.0, height_buffer = 2000.0, thickness = 50.0):
    height_a, _ = sample_height(world, loc_a.x, loc_a.y)
    height_b, _ = sample_height(world, loc_b.x, loc_b.y)
    preview_height = max(height_a, height_b) + height_buffer
    color = [1.0, 0.0, 1.0, 1.0]
    corner_1 = unreal.Vector(min_x, min_y, preview_height)
    corner_2 = unreal.Vector(max_x, min_y, preview_height)
    corner_3 = unreal.Vector(max_x, max_y, preview_height)
    corner_4 = unreal.Vector(min_x, max_y, preview_height)
    unreal.SystemLibrary.draw_debug_line(world, corner_1, corner_2, color, duration, thickness)
    unreal.SystemLibrary.draw_debug_line(world, corner_2, corner_3, color, duration, thickness)
    unreal.SystemLibrary.draw_debug_line(world, corner_3, corner_4, color, duration, thickness)
    unreal.SystemLibrary.draw_debug_line(world, corner_4, corner_1, color, duration, thickness)

#Traversability functions
#Traversability method based on A* adapted as per: Xu, Shi, Yin & Peng (2025) PARC
#Sample height on the landscape using line trace
def sample_height(world, x, y):
    #Set start and end points for line trace
    start = unreal.Vector(x, y, 100000.0)
    end = unreal.Vector(x, y, -100000.0)
    #Line trace
    hit = unreal.SystemLibrary.line_trace_single(world, start, end, unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,
                                                 False, [], unreal.DrawDebugTrace.NONE, True)
    #Save result as tuple
    result_tuple = hit.to_tuple()
    #Extract hit result [True/Flase] from [0] and hit location from [4]
    hit_result = result_tuple[0]
    location = result_tuple[4]
    normal = result_tuple[6]
    #Check results and return valid height
    if hit_result == False:
        return None, None
    return location.z, normal

#Convert from cell to world
def cell_to_world(row, col, min_x, min_y, cell_size):
    x = min_x + cell_size * col + cell_size/2
    y = min_y + cell_size * row + cell_size/2
    return x, y

#Function to turn drawn area into a region for pipeline
def build_region_from_brush(locations, radii):
    circles = []
    for loc, rad in zip(locations, radii):
        circles.append(Point(loc.x, loc.y).buffer(rad))
    combined_region = shapely.unary_union(circles)
    return combined_region

#Create a grid over the region to be tested for traversability (reduce number of nodes in graph)
#Cell size decides how coarse the grid is, defaulted to 200 as per tests (1ms speed and catches cliffs)
def build_grid(world, min_x, min_y, max_x, max_y, cell_size = 200.0):
    num_cols = int((max_x - min_x) / cell_size)
    num_rows = int((max_y - min_y) / cell_size)
    heights = {}
    normals = {}
    for row in range(num_rows):
        for col in range(num_cols):
            x, y = cell_to_world(row, col, min_x, min_y, cell_size)
            height, normal = sample_height(world, x, y)
            heights[(row,col)] = height
            normals[(row,col)] = normal
    return heights, normals

#Build grid - version for brush tool
def build_grid_brush(world, region, cell_size = 200.0):
    min_x, min_y, max_x, max_y = region.bounds
    num_cols = int((max_x - min_x) / cell_size)
    num_rows = int((max_y - min_y) / cell_size)
    heights = {}
    normals = {}
    for row in range(num_rows):
        for col in range(num_cols):
            x, y = cell_to_world(row, col, min_x, min_y, cell_size)
            #Check if the area is actually inside the drawn region
            if region.contains(Point(x, y)):
                height, normal = sample_height(world, x, y)
                heights[(row,col)] = height
                normals[(row,col)] = normal
    return heights, normals, min_x, min_y

#Given a grid cell, return all neighbouring cells that can be walked to
def get_walk_neighbours(cell_position, heights, normals, cell_size, walk_angle = 44.7):
    #Calculate max allowable height for a neighbouring grid to count as walkable
    walk_height_limit = math.tan(math.radians(walk_angle)) * cell_size
    #Loop through 8 surrounding blocks and store which are walkable
    walkable = []
    row, col = cell_position
    directions = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
    for dr, dc in directions:
        row_n = row + dr
        col_n = col + dc
        if (row_n, col_n) in heights:
            if heights[cell_position] != None and heights[(row_n, col_n)] != None:
                height_diff = abs(heights[cell_position] - heights[(row_n, col_n)])
                if height_diff < walk_height_limit:
                    #Also check the neighbouring cell's own cached surface normal isn't too steep to stand on
                    normal = normals[(row_n, col_n)]
                    up_vector = unreal.Vector(0, 0, 1)
                    dot = normal.x * up_vector.x + normal.y * up_vector.y + normal.z * up_vector.z
                    surface_angle = math.degrees(math.acos(dot))
                    if surface_angle < walk_angle:
                        walkable.append((row_n, col_n))
    return walkable

#Calculate jump stats for jump check (defaults from UE5.7 3rd Person Character Movement Component)
def calculate_jump_stats(jump_z_velocity = 500.0, gravity = -980.0, max_walk_speed = 500.0, jump_buffer = 1.1):
    jump_height = jump_z_velocity**2 / (2.0 * -gravity) * jump_buffer
    airtime = (2.0 * jump_z_velocity) / -gravity
    jump_distance = max_walk_speed * airtime * jump_buffer
    return jump_height, jump_distance

#Function to check gap size vs jump (rules out possibility of jump)
#Note: this function is not using physics simulation and is only as sophiticated as basic distance checks
#Character variables need to be set in tool
def check_gap(point_a, point_b, char_jump_dist, char_jump_height, landing_normal, walk_angle, safe_fall_dist = 500.0):
    v1 = point_a
    v2 = point_b
    #Horizontal distance
    horizontal_distance = ((v1.x - v2.x)**2 + (v1.y - v2.y)**2)**0.5
    #Early out on horizontal distance first, cheapest check
    if(horizontal_distance > char_jump_dist):
        return False
    #Check landing spot isn't too steep to actually stand on
    up_vector = unreal.Vector(0, 0, 1)
    dot = landing_normal.x * up_vector.x + landing_normal.y * up_vector.y + landing_normal.z * up_vector.z
    landing_angle = math.degrees(math.acos(dot))
    if landing_angle > walk_angle:
        return False
    #Vertical delta, positive means jumping up, negative means dropping down
    vertical_delta = v2.z - v1.z
    #Jumping up, check against jump height
    if(vertical_delta > 0):
        return vertical_delta < char_jump_height
    #Dropping down, check against separate safe fall distance
    else:
        return abs(vertical_delta) < safe_fall_dist

#Given a grid cell, retrun all cells that can be jumped to
def get_jump_neighbours(cell_position, heights, normals, cell_size, min_x, min_y, jump_height, 
                        jump_distance, walk_angle, safe_fall_distance = 500.0):
    jump_radius = int(jump_distance/cell_size) + 1
    jumpable = []
    row, col = cell_position
    for dr in range(-jump_radius, jump_radius + 1):
        for dc in range(-jump_radius, jump_radius + 1):
            row_j = row + dr
            col_j = col + dc
            if abs(dr) <= 1 and abs(dc) <= 1:
                continue
            if (row_j, col_j) in heights:
                if heights[cell_position] != None and heights[(row_j, col_j)] != None:
                    x, y = cell_to_world(row, col, min_x, min_y, cell_size)
                    x_j, y_j = cell_to_world(row_j, col_j, min_x, min_y, cell_size)
                    pos_current = unreal.Vector(x, y, heights[cell_position])
                    pos_candidate = unreal.Vector(x_j, y_j, heights[(row_j, col_j)])
                    landing_normal = normals[(row_j, col_j)]
                    jump_check = check_gap(pos_current, pos_candidate, jump_distance, jump_height, landing_normal, walk_angle, safe_fall_distance)
                    if jump_check == True:
                        jumpable.append((row_j, col_j))
    return jumpable

#Build graph for A*
def build_graph(heights, normals, cell_size, min_x, min_y, jump_z_velocity = 500.0, gravity = -980.0, max_walk_speed = 500.0,
               safe_fall_distance = 500.0, jump_buffer = 1.1, walk_angle = 44.7, jump_cost_multiplier = 40.0):
    #Create empty graph and add all cells as nodes
    graph = nx.DiGraph()
    for key in heights:
        graph.add_node(key)
    jump_height, jump_distance = calculate_jump_stats(jump_z_velocity, gravity, max_walk_speed, jump_buffer)
    #Add egdes to graph (cost function adapter as per PARC), stochastic function removed (not needed)
    for cell in heights:
        walkable = get_walk_neighbours(cell, heights, normals, cell_size, walk_angle)
        jumpable = get_jump_neighbours(cell, heights, normals, cell_size, min_x, min_y,
                                      jump_height, jump_distance, walk_angle, safe_fall_distance)
        row, col = cell
        x, y = cell_to_world(row, col, min_x, min_y, cell_size)
        z = heights[cell]
        for neighbour in walkable:
            row_n, col_n = neighbour
            x_n, y_n = cell_to_world(row_n, col_n, min_x, min_y, cell_size)
            z_n = heights[neighbour]
            cost = 1.0 * ((x - x_n)**2 + (y - y_n)**2) + 0.15 * (z - z_n)**2
            graph.add_edge(cell, neighbour, weight=cost)
        for target in jumpable:
            row_t, col_t = target
            x_t, y_t = cell_to_world(row_t, col_t, min_x, min_y, cell_size)
            z_t = heights[target]
            #Jump cost is multiplied by a parameter to ensure it is not favoured over walking
            cost = (1.0 * ((x - x_t)**2 + (y - y_t)**2) + 0.15 * (z - z_t)**2) * jump_cost_multiplier
            graph.add_edge(cell, target, weight=cost)
    return graph

#Heuristic function (not provided by PARC but using straight line distance)
def grid_heuristic(node_a, node_b, min_x, min_y, cell_size):
    r1, c1 = node_a
    r2, c2 = node_b
    x1, y1 = cell_to_world(r1, c1, min_x, min_y, cell_size)
    x2, y2 = cell_to_world(r2, c2, min_x, min_y, cell_size)
    return (x1 - x2)**2 + (y1 - y2)**2

#Apply A* from networkx to find path
def find_path(graph, start_node, end_node, min_x, min_y, cell_size):
    #Use a closure as nx.astar_path's heuristic expects a fucntion with only two arguments
    def heuristic_closure(node_a, node_b):
        return grid_heuristic(node_a, node_b, min_x, min_y, cell_size)
    #Call A* star using wight assigned in build_graph, use try to avoid nx.NetworkXNoPath error and crash
    try:
        path = nx.astar_path(graph, start_node, end_node, heuristic=heuristic_closure, weight='weight')
        return path
    except nx.NetworkXNoPath:
        return None

#Fine check for landing stability
def check_landing_stability(world, landing_point, walk_angle, offset = 35.0, min_required = 2):
    offsets = [(offset, 0), (-offset, 0), (0, offset), (0, -offset)]
    pass_count = 0
    for dx, dy in offsets:
        h, normal = sample_height(world, landing_point.x + dx, landing_point.y + dy)
        if h is None:
            continue
        up_vector = unreal.Vector(0, 0, 1)
        dot = normal.x * up_vector.x + normal.y * up_vector.y + normal.z * up_vector.z
        angle = math.degrees(math.acos(dot))
        if angle <= walk_angle:
            pass_count += 1
    return pass_count >= min_required


#Check if path is feasible
def validate_path(path, world, walk_angle, min_x, min_y, cell_size, heights):
    previous_was_jump = False
    #Loop through paths
    for node in range(1, len(path)):
        current_node = path[node]
        previous_node = path[node - 1]
        row, col = current_node
        row_p, col_p = previous_node
        row_diff = row - row_p
        col_diff = col - col_p
        #Check if jump
        if abs(row_diff) > 1 or abs(col_diff) > 1:
            x, y = cell_to_world(row, col, min_x, min_y, cell_size)
            z = heights[current_node]
            landing_point = unreal.Vector(x, y, z)
            #Check stability
            stable = check_landing_stability(world, landing_point, walk_angle)
            if stable == False:
                return False
            previous_was_jump = True
        else:
            previous_was_jump = False
    return True

#Attempts to find a path, and if invalid due to a bad jump chain, removes those edges and retries
def find_valid_path(graph, start_node, end_node, min_x, min_y, cell_size, heights, world, walk_angle, max_retries = 10):
    current_graph = graph.copy()
    for attempt in range(max_retries):
        path = find_path(current_graph, start_node, end_node, min_x, min_y, cell_size)
        if path is None:
            return None
        previous_was_jump = False
        chain_edges = []
        failed_chain = None
        for i in range(1, len(path)):
            curr = path[i]
            prev = path[i-1]
            is_jump = abs(curr[0]-prev[0]) > 1 or abs(curr[1]-prev[1]) > 1
            if is_jump:
                x, y = cell_to_world(curr[0], curr[1], min_x, min_y, cell_size)
                z = heights[curr]
                landing_point = unreal.Vector(x, y, z)
                stable = check_landing_stability(world, landing_point, walk_angle)
                chain_edges.append((prev, curr))
                if not stable and failed_chain is None:
                    failed_chain = list(chain_edges)
            else:
                chain_edges = []
        if failed_chain is None:
            return path
        for edge in failed_chain:
            if current_graph.has_edge(edge[0], edge[1]):
                current_graph.remove_edge(edge[0], edge[1])
    return None


#Runs the full traversability pipeline end to end, from two named POIs to a final result
def run_traversability_check(poi_a_label, poi_b_label, pad_left, pad_right, pad_up, pad_down,
                             cell_size = 200.0, jump_z_velocity = 500.0, gravity = -980.0,
                             max_walk_speed = 500.0, safe_fall_distance = 500.0, jump_buffer = 1.1,
                             walk_angle = 44.7):
    #Start timer
    time_start = time.perf_counter()

    #Get references to world, EAS, actor list and location data
    world = unreal.EditorLevelLibrary.get_editor_world()
    EAS = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    ActorList = EAS.get_all_level_actors()
    poi_a = unreal.EditorFilterLibrary.by_actor_label(ActorList, poi_a_label)[0]
    poi_b = unreal.EditorFilterLibrary.by_actor_label(ActorList, poi_b_label)[0]
    loc_a = poi_a.get_actor_location()
    loc_b = poi_b.get_actor_location()
    min_x, max_x, min_y, max_y = calculate_region_bounds(loc_a, loc_b, pad_left, pad_right, pad_up, pad_down)

    #Build the height/normal grid and the searchable graph for this region
    heights, normals = build_grid(world, min_x, min_y, max_x, max_y, cell_size)
    graph = build_graph(heights, normals, cell_size, min_x, min_y, jump_z_velocity, gravity,
                        max_walk_speed, safe_fall_distance, jump_buffer, walk_angle)

    #Convert the two POIs' real world positions into grid node coordinates
    start_col = int((loc_a.x - min_x) / cell_size)
    start_row = int((loc_a.y - min_y) / cell_size)
    end_col = int((loc_b.x - min_x) / cell_size)
    end_row = int((loc_b.y - min_y) / cell_size)
    start_node = (start_row, start_col)
    end_node = (end_row, end_col)

    #Run A* to find a path, retrying if an invalid jump chain is found
    path = find_valid_path(graph, start_node, end_node, min_x, min_y, cell_size, heights, world, walk_angle)
    path_valid = path is not None

    #If a path was found, draw it in the viewport as green spheres
    if path is not None and path_valid:
        for (r, c) in path:
            x, y = cell_to_world(r, c, min_x, min_y, cell_size)
            z = heights[(r, c)]
            point = unreal.Vector(x, y, z)
            unreal.SystemLibrary.draw_debug_sphere(world, point, 100.0, 12, [0.0, 1.0, 0.0, 1.0], 60.0, 5.0)

    #End timer
    time_end = time.perf_counter()
    total_time = time_end - time_start

    #Capture grid
    grid_size = len(heights)

    #Return whether a path was found AND validated stable, the path itself, elapsed time and grid size
    return (path is not None and path_valid), path, total_time, grid_size


#Brush version of final function
#Runs the full traversability pipeline end to end, using a brush-drawn region instead of padding
def run_traversability_check_brush(poi_a_label, poi_b_label, locations, radii,
                             cell_size = 200.0, jump_z_velocity = 500.0, gravity = -980.0,
                             max_walk_speed = 500.0, safe_fall_distance = 500.0, jump_buffer = 1.1,
                             walk_angle = 44.7):
    #Start timer
    time_start = time.perf_counter()

    #Get references to world, EAS, actor list and location data
    world = unreal.EditorLevelLibrary.get_editor_world()
    EAS = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    ActorList = EAS.get_all_level_actors()
    poi_a = unreal.EditorFilterLibrary.by_actor_label(ActorList, poi_a_label)[0]
    poi_b = unreal.EditorFilterLibrary.by_actor_label(ActorList, poi_b_label)[0]
    loc_a = poi_a.get_actor_location()
    loc_b = poi_b.get_actor_location()

    #Brush version: build the region from the brush stroke instead of padding bounds
    region = build_region_from_brush(locations, radii)

    #VBrush version: build the height/normal grid using the brush-based version, which also returns min_x/min_y
    heights, normals, min_x, min_y = build_grid_brush(world, region, cell_size)

    graph = build_graph(heights, normals, cell_size, min_x, min_y, jump_z_velocity, gravity,
                        max_walk_speed, safe_fall_distance, jump_buffer, walk_angle)

    #Convert the two POIs' real world positions into grid node coordinates
    start_col = int((loc_a.x - min_x) / cell_size)
    start_row = int((loc_a.y - min_y) / cell_size)
    end_col = int((loc_b.x - min_x) / cell_size)
    end_row = int((loc_b.y - min_y) / cell_size)
    start_node = (start_row, start_col)
    end_node = (end_row, end_col)

    #Run A* to find a path, retrying if an invalid jump chain is found
    path = find_valid_path(graph, start_node, end_node, min_x, min_y, cell_size, heights, world, walk_angle)
    path_valid = path is not None

    #If a path was found, draw it in the viewport as green spheres
    if path is not None and path_valid:
        for (r, c) in path:
            x, y = cell_to_world(r, c, min_x, min_y, cell_size)
            z = heights[(r, c)]
            point = unreal.Vector(x, y, z)
            unreal.SystemLibrary.draw_debug_sphere(world, point, 100.0, 12, [0.0, 1.0, 0.0, 1.0], 60.0, 5.0)

    #End timer
    time_end = time.perf_counter()
    total_time = time_end - time_start

    #Capture grid
    grid_size = len(heights)

    #Return whether a path was found AND validated stable, the path itself, elapsed time and grid size
    return (path is not None and path_valid), path, total_time, grid_size
