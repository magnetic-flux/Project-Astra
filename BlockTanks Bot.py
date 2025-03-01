from selenium import webdriver
import time, keyboard, heapq, math, shapely.geometry as geo

# --- CONSTANTS ---
debugging = True
pathfinding_frequency = 1
threat_range = 10
dodging_time_interval = 1 # Seconds forward to predict game state
bounce_check_distance = 0.1 # Increment in blocks that powerup trajectory is analyzed to check for collisions with walls; lower values are more accurate but slower
get_wall_edge_algorithm_iterations = 10 # Number of iterations in get_wall_edge_between algorithm; higher values are more accurate but slower
get_wall_edge_algorithm_x_size = 0.05 # Size in blocks of diagonal X drawn by get_wall_edge_between algorithm; optimal value to be experimentally determined

# --- VARIABLE INITIALIZATION ---
pathfinding_counter = 0
pixels_per_block = 88
tank_size_in_blocks = 0.5
tank_speed = 2.5
bullet_speed_conversion_factor = 0.4655 # Multiply by pixel distance from scraped previous position to get blocks per second
threat_range_in_pixels = threat_range * pixels_per_block
bounce_check_distance_in_pixels = bounce_check_distance * pixels_per_block
get_wall_edge_algorithm_x_size_in_pixels = get_wall_edge_algorithm_x_size * pixels_per_block

regular_bullet_speed = 4.25
regular_bullet_radius = 0.1
rapid_bullet_speed = 7.25
grenade_detonation_time = 1.5
grenade_blast_radius = 2
grenade_starting_rotation_rate = 0.16974
bottle_bomb_detonation_time = 1.5
bottle_bomb_blast_radius = 1
bottle_bomb_bubble_radius = 1
bottle_bomb_starting_rotation_rate = grenade_starting_rotation_rate
bomb_rotation_increase_rate = 0.05 # Angle per frame increase per bounce
bomb_bounce_threshold = 5
bomb_speed = 3.5
bomb_diameter = 0.45
bomb_blast_radius = 3
rocket_blast_radius = 1.25
rocket_diameter = 0.5
rocket_speed = 7.25
sniper_warmup_time = 0.5
shotgun_speed = 7.25
minigun_speed = 7.25

coordinate_changes = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
directions = ["right", "downright", "down", "downleft", "left", "upleft", "up", "upright"]

js_script_tanks = """
let tanks = window.game.world.children.filter(obj => obj.key === 'body b' || obj.key === 'body r');
let tank_data = tanks.map(tank => {
    return {
        team: tank.key === 'body b' ? 'blue' : 'red',
        x: Math.round(tank.worldPosition.x),
        y: Math.round(tank.worldPosition.y)
    };
});
return tank_data;
"""

js_script_projectiles_r = """
let bullet_keys = ['bullet r', 'rocketBullet r', 'grenadeBullet r', 'bottleBombBullet r', 'bubble r_0'];
let sniper_key = 'sniperShoot_r';
let bomb_key = 'volcanoBullet r';

let projectiles = window.game.world.children.filter(obj => bullet_keys.includes(obj.key) || obj.key === sniper_key);
let projectile_data = projectiles.map(proj => {
    let data = {
        type: proj.key.replace(/[\s_]?r(_0)?$/, ''),
        x: Math.round(proj.worldPosition.x),
        y: Math.round(proj.worldPosition.y),
        prev_x: proj.previousPosition ? proj.previousPosition.x : null,
        prev_y: proj.previousPosition ? proj.previousPosition.y : null
    };
    if (proj.key === sniper_key || proj.key === bomb_key) {
        data.rotation = proj.worldRotation;
        data.prev_rotation = proj.previousRotation !== undefined ? proj.previousRotation : null;
    }
    return data;
});
return projectile_data;
"""

js_script_projectiles_b = """
let bullet_keys = ['bullet b', 'rocketBullet b', 'grenadeBullet b', 'bottleBombBullet b', 'bubble b_0'];
let sniper_key = 'sniperShoot_b';
let bomb_key = 'volcanoBullet b';

let projectiles = window.game.world.children.filter(obj => bullet_keys.includes(obj.key) || obj.key === sniper_key);
let projectile_data = projectiles.map(proj => {
    let data = {
        type: proj.key.replace(/[\s_]?r(_0)?$/, ''),
        x: Math.round(proj.worldPosition.x),
        y: Math.round(proj.worldPosition.y),
        prev_x: proj.previousPosition ? proj.previousPosition.x : null,
        prev_y: proj.previousPosition ? proj.previousPosition.y : null
    };
    if (proj.key === sniper_key || proj.key === bomb_key) {
        data.rotation = proj.worldRotation;
        data.prev_rotation = proj.previousRotation !== undefined ? proj.previousRotation : null;
    }
    return data;
});
return projectile_data;
"""

js_script_walls = """
let mapGroup = window.game.world.children.find(obj => obj.name === 'mapGroup');
if (!mapGroup) return [];

let walls = mapGroup.children.map(tile => {
    return {
        x: Math.round(tile.worldPosition.x),
        y: Math.round(tile.worldPosition.y),
        prev_x: tile.previousPosition ? Math.round(tile.previousPosition.x) : null,
        prev_y: tile.previousPosition ? Math.round(tile.previousPosition.y) : null
    };
});
return walls;
"""

js_script_fences = """
let mapGroupSecondary = window.game.world.children.find(obj => obj.name === 'mapGroupSecondary');
if (!mapGroupSecondary) return [];

let fences = mapGroupSecondary.children.filter(tile => tile.key === 'bulletOnlyTile').map(tile => {
    return {
        x: Math.round(tile.worldPosition.x),
        y: Math.round(tile.worldPosition.y),
        prev_x: tile.previousPosition ? Math.round(tile.previousPosition.x) : null,
        prev_y: tile.previousPosition ? Math.round(tile.previousPosition.y) : null
    };
});
return fences;
"""

js_script_camera = """
return {
    width: window.game.camera.view.width,
    height: window.game.camera.view.height
};
"""

# --- FUNCTIONS ---
def get_enemy_tanks(): return [tank for tank in driver.execute_script(js_script_tanks) if tank["team"] != team]

def get_enemy_projectiles(): return driver.execute_script(js_script_projectiles_r if team == "blue" else js_script_projectiles_b)

def get_walls(): return driver.execute_script(js_script_walls)

def get_fences(): return driver.execute_script(js_script_fences)

def get_screen_center():
    camera_dimensions = driver.execute_script(js_script_camera)
    print("Camera dimensions:", (camera_dimensions["width"], camera_dimensions["height"]))
    return (camera_dimensions["width"] / 2, camera_dimensions["height"] / 2)

def get_self_tank(): return min(driver.execute_script(js_script_tanks), key=lambda tank: (tank["x"] - screen_center[0])**2 + (tank["y"] - screen_center[1])**2)

def detect_team():
    global team, screen_center, debugging
    screen_center = get_screen_center()
    self_tank = get_self_tank()
    team = self_tank["team"]
    if debugging: print(f"Detected team: {team}")

def get_grid_coordinates_of_wall(wall): return (math.ceil((wall["x"] / pixels_per_block)), math.ceil((wall["y"] / pixels_per_block)))

def get_grid_coordinates_of_tank(tank):
    global walls, debugging
    decimal_grid_coordinates = (walls[0]["x"] / pixels_per_block, walls[0]["y"] / pixels_per_block)
    reference_grid_coordinates = get_grid_coordinates_of_wall(walls[0])
    offset = (reference_grid_coordinates[0] - decimal_grid_coordinates[0], reference_grid_coordinates[1] - decimal_grid_coordinates[1])
    if debugging: print("Offset:", offset)
    return (math.floor((tank["x"] / pixels_per_block) + offset[0]), math.floor((tank["y"] / pixels_per_block) + offset[1]))

def astar(start, goal, obstacles):
    def h(a, b): return max(abs(a[0] - b[0]), abs(a[1] - b[1]))

    def reconstruct_path(came_from, current):
        global coordinate_changes, directions
        while current in came_from:
            if came_from[current] == start: return directions[coordinate_changes.index((current[0] - came_from[current][0], current[1] - came_from[current][1]))]
            current = came_from[current]

    def neighbors(current):
        valid_neighbors = []
        for coordinate_change in [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]:
            neighbor = (current[0] + coordinate_change[0], current[1] + coordinate_change[1])
            if not (neighbor in obstacles): valid_neighbors.append(neighbor)
        return valid_neighbors

    astar_start_time = time.time()
    
    open_set, came_from, g_scores, f_scores = [], {}, {}, {} # g_score[n] is the cheapest path from start to n currently known; f_score[n] = g_score[n] + h(n)
    g_scores[start], f_scores[start] = 0, h(start, goal)

    heapq.heappush(open_set, (f_scores[start], start))

    while open_set:
        if time.time() - astar_start_time > 0.25: return "timeout"
        
        current = heapq.heappop(open_set)[1]
        if current == goal: return reconstruct_path(came_from, current)

        for neighbor in neighbors(current):
            tentative_g_score = g_scores[current] + 1
            if tentative_g_score < g_scores.get(neighbor, float("inf")):
                came_from[neighbor], g_scores[neighbor], f_scores[neighbor] = current, tentative_g_score, tentative_g_score + h(neighbor, goal)
                if neighbor not in open_set: heapq.heappush(open_set, (f_scores[neighbor], neighbor))

    return "none"

def is_in_wall(coordinates):
    global walls
    for wall in walls:
        if wall["x"] <= coordinates[0] <= wall["x"] + pixels_per_block and wall["y"] <= coordinates[1] <= wall["y"] + pixels_per_block: return True
    return False

def get_wall_edge_between(point1, point2): # point1 must be outside the wall, point2 must be inside the wall
    global get_wall_edge_algorithm_iterations, get_wall_edge_algorithm_x_size, debugging
    
    outside_point, inside_point = point1, point2
    for i in range(get_wall_edge_algorithm_iterations):
        middle_point = ((outside_point[0] + inside_point[0]) / 2, (outside_point[1] + inside_point[1]) / 2)
        if is_in_wall(middle_point): inside_point = middle_point
        else: outside_point = middle_point
    bounce_point = ((outside_point[0] + inside_point[0]) / 2, (outside_point[1] + inside_point[1]) / 2)

    corners_inside_walls = []
    for sign in [(-1, -1), (1, -1), (-1, 1), (1, 1)]: corners_inside_walls.append(is_in_wall([bounce_point[0] + (sign[0] * get_wall_edge_algorithm_x_size_in_pixels), bounce_point[1] + (sign[1] * get_wall_edge_algorithm_x_size_in_pixels)]))

    if corners_inside_walls == [True, True, False, False] or corners_inside_walls == [False, False, True, True]: return ("horizontal", bounce_point)
    if corners_inside_walls == [True, False, True, False] or corners_inside_walls == [False, True, False, True]: return ("vertical", bounce_point)
    else: return ("failure", bounce_point)

def distance(point1, point2): return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def distance_to_self_center(coordinates):
    self_tank = get_self_tank()
    return distance((self_tank["x"], self_tank["y"]), coordinates)

def get_trajectory_linestring_points(bullet, max_bounces, is_sniper):
    global walls, debugging
    if not is_sniper:
        previous_position_distance = distance((bullet["prev_x"], bullet["prev_y"]), (bullet["x"], bullet["y"]))
        bullet_speed = previous_position_distance * bullet_speed_conversion_factor
        total_path_length = regular_bullet_speed * dodging_time_interval * pixels_per_block if abs(bullet_speed - regular_bullet_speed) < abs(bullet_speed - rapid_bullet_speed) else rapid_bullet_speed * dodging_time_interval * pixels_per_block # in pixels
        dx, dy = (bounce_check_distance_in_pixels / previous_position_distance)*(bullet["x"] - bullet["prev_x"]), (bounce_check_distance_in_pixels / previous_position_distance)*(bullet["y"] - bullet["prev_y"])
    else:
        total_path_length = float('inf')
        if bullet["rotation"] < 0: actual_rotation = -bullet["rotation"] # Above the horizontal
        else: actual_rotation = 2 * math.pi - bullet["rotation"] # Below the horizontal
        if actual_rotation < 0 or actual_rotation > 2 * math.pi:
            print("CAUTION: Calculated sniper rotation angle not between 0 to 2pi")
            return "failure"
        
        dx, dy = math.cos(actual_rotation) * bounce_check_distance_in_pixels, math.sin(actual_rotation) * bounce_check_distance_in_pixels

    linestring_points = [(bullet["x"], bullet["y"])]
    next_point = (bullet["x"], bullet["y"])
    bounces = 0

    for i in range(1, int(total_path_length / bounce_check_distance_in_pixels) + 1):
        current_point = next_point
        next_point = (current_point[0] + dx, current_point[1] + dy)
        if is_in_wall(next_point):
            linestring_points.append(current_point)
            if bounces >= max_bounces: break
            else:
                bounces += 1
                get_wall_edge_output = get_wall_edge_between(current_point, next_point)
                if get_wall_edge_output[0] == "horizontal": dx, dy = dx, -dy
                elif get_wall_edge_output[0] == "vertical": dx, dy = -dx, dy
                else: print("CAUTION: Detected bounce but get_wall_edge_between algorithm failed to locate wall face direction")
                current_point = next_point
                next_point = (current_point[0] + dx, current_point[1] + dy)
    
    linestring_points.append(current_point)
    return linestring_points

def dodge():
    global walls, enemy_projectiles, coordinate_changes, directions, dodging_time_interval, debugging
    bullets, grenades, bottle_bombs, bombs, rockets, snipers, geo_walls = [], [], [], [], [], [], []

    for wall in walls: geo_walls.append(geo.box(wall["x"], wall["y"], wall["x"] + pixels_per_block, wall["y"] + pixels_per_block))

    for projectile in enemy_projectiles:
        if distance_to_self_center((projectile["x"], projectile["y"])) < threat_range_in_pixels:
            if projectile["type"] == "bullet":
                bullet_linestring_points = get_trajectory_linestring_points(projectile, 1, False)
                if bullet_linestring_points != "failure":
                    for i in range(0, len(bullet_linestring_points) - 1):
                        bullet_linestring = geo.LineString([bullet_linestring_points[i], bullet_linestring_points[i + 1]])
                        bullet_linestring.buffer(regular_bullet_radius)
                        bullets.append(bullet_linestring)    

    is_first_region = True
    for region_type in [bullets, grenades, bottle_bombs, bombs, rockets, snipers, geo_walls]:
        for region in region_type:
            if is_first_region:
                region_to_avoid = geo.GeometryCollection([region])
                is_first_region = False
            else: region_to_avoid = region_to_avoid.union(region)
    
    if debugging:
        fig = plt.figure()
        ax = fig.add_subplot()

        # Plot bullet linestring points
        for bullet in bullets:
            bullet_linestring_points = list(bullet.coords)
            x_bullet_points = [point[0] for point in bullet_linestring_points]
            y_bullet_points = [-point[1] for point in bullet_linestring_points]
            ax.scatter(x_bullet_points, y_bullet_points, color='red', label='Bullet Points')

        # Plot wall coordinates
        x_wall_points = [wall["x"] for wall in walls]
        y_wall_points = [-wall["y"] for wall in walls]
        ax.scatter(x_wall_points, y_wall_points, color='blue', label='Wall Points')

        plt.legend()
        plt.show()

    return "none"

# --- MAIN ---
if debugging: import matplotlib.pyplot as plt

driver = webdriver.Chrome()
driver.get("https://blocktanks.net")
paused = True

while True:
    main_loop_start_time = time.time()
    
    if keyboard.is_pressed('p'):
        while keyboard.is_pressed('p'):
            pass
        paused = not paused
        print("Paused" if paused else "Unpaused")
        detect_team()

    if not paused and not (keyboard.is_pressed('w') or keyboard.is_pressed('a') or keyboard.is_pressed('s') or keyboard.is_pressed('d')):        
        
        # --- PATHFINDING ---
        pathfinding_counter += 1
        if pathfinding_counter >= pathfinding_frequency:
            pathfinding_start_time = time.time()
            
            walls = get_walls()
            fences = get_fences()
            wall_grid_coordinates = []
            for wall in walls: wall_grid_coordinates.append(get_grid_coordinates_of_wall(wall))
            for wall in fences: wall_grid_coordinates.append(get_grid_coordinates_of_wall(wall))

            enemy_tanks = get_enemy_tanks()
            if enemy_tanks != []:
                closest_enemy = min(enemy_tanks, key=lambda tank: (tank["x"] - screen_center[0])**2 + (tank["y"] - screen_center[1])**2)
                closest_enemy_grid_coordinates = get_grid_coordinates_of_tank(closest_enemy)
                self_grid_coordinates = get_grid_coordinates_of_tank(get_self_tank())

                # Removing all walls with x = -1 or y = maximum y, I have no idea why a boundary layer exists for these two sides
                max_y = max(wall[1] for wall in wall_grid_coordinates)
                for wall in wall_grid_coordinates:
                    if wall[0] == -1 or wall[1] == max_y: wall_grid_coordinates.remove(wall)                

                goal_direction = astar(self_grid_coordinates, closest_enemy_grid_coordinates, wall_grid_coordinates)
                
                if goal_direction != "timeout" and goal_direction != None: print("A-star algorithm wants to go " + goal_direction)
                else: print("CAUTION: A-star algorithm timed out")

                if debugging:
                    print("Self grid coordinates:", self_grid_coordinates)
                    print("Closest enemy grid coordinates:", closest_enemy_grid_coordinates)
                    # print("Wall coordinates:", wall_grid_coordinates)

                    # x_wall_coordinates, y_wall_coordinates = [wall[0] for wall in wall_grid_coordinates], [-wall[1] for wall in wall_grid_coordinates]
                    # plt.scatter(x_wall_coordinates, y_wall_coordinates, color="black")
                    # plt.scatter(self_grid_coordinates[0], -self_grid_coordinates[1], color="blue")
                    # plt.scatter(closest_enemy_grid_coordinates[0], -closest_enemy_grid_coordinates[1], color="red")
                    # plt.show()

            elif debugging: print("No enemy tanks detected")
            
            pathfinding_counter = 0

            pathfinding_end_time = time.time()
            if debugging: print("Pathfinding time:", pathfinding_end_time - pathfinding_start_time, "seconds")
        
        # --- DODGING ---
        dodging_start_time = time.time()
        
        enemy_projectiles = get_enemy_projectiles()
        walls = get_walls()

        dodging_direction = dodge()

        if dodging_direction == "none": final_direction = goal_direction
        else: final_direction = dodging_direction

        dodging_end_time = time.time()

        if debugging: print("Dodging time:", dodging_end_time - dodging_start_time, "seconds")

        # --- MOVEMENT ---
        
        # TODO: Implement movement
        
        # --- NUTS AND BOLTS ---
        if debugging:
            # print("Enemy tanks:", enemy_tanks)
            print("Enemy projectiles:", enemy_projectiles)
            # print("Walls:", walls)
            # print("Fences:", fences)

        main_loop_end_time = time.time()
        main_loop_time = main_loop_end_time - main_loop_start_time
        if main_loop_time > 0.25: print("CAUTION: Unusually long main loop iteration time (" + str(main_loop_time) + " seconds)")
        if debugging: print()
            
        time.sleep(2)
