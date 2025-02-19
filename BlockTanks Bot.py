from selenium import webdriver
import time, keyboard, heapq, math

# --- CONSTANTS ---
debugging = True
pathfinding_frequency = 1

# --- VARIABLE INITIALIZATION ---
pathfinding_counter = 0
pixels_per_block = 88
tank_size_in_blocks = 0.5
tank_speed = 2.5
bullet_speed_conversion_factor = 0.4655 # Multiply by pixel distance from scraped previous position to get blocks per second

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
        type: proj.key,
        x: Math.round(proj.worldPosition.x),
        y: Math.round(proj.worldPosition.y),
        prev_x: proj.previousPosition ? Math.round(proj.previousPosition.x) : null,
        prev_y: proj.previousPosition ? Math.round(proj.previousPosition.y) : null
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
        type: proj.key,
        x: Math.round(proj.worldPosition.x),
        y: Math.round(proj.worldPosition.y),
        prev_x: proj.previousPosition ? Math.round(proj.previousPosition.x) : null,
        prev_y: proj.previousPosition ? Math.round(proj.previousPosition.y) : null
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
def get_enemy_tanks():
    return [tank for tank in driver.execute_script(js_script_tanks) if tank["team"] != team]

def get_enemy_projectiles():
    return driver.execute_script(js_script_projectiles_r if team == "blue" else js_script_projectiles_b)

def get_walls():
    return driver.execute_script(js_script_walls)

def get_fences():
    return driver.execute_script(js_script_fences)

def get_screen_center():
    camera_dimensions = driver.execute_script(js_script_camera)
    print("Camera dimensions:", (camera_dimensions["width"], camera_dimensions["height"]))
    return (camera_dimensions["width"] / 2, camera_dimensions["height"] / 2)

def get_self_tank():
    tanks = driver.execute_script(js_script_tanks)
    return min(tanks, key=lambda tank: (tank["x"] - screen_center[0])**2 + (tank["y"] - screen_center[1])**2)

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
        while current in came_from:
            if came_from[current] == start:
                coordinate_changes = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
                directions = ["right", "downright", "down", "downleft", "left", "upleft", "up", "upright"]
                return directions[coordinate_changes.index((current[0] - came_from[current][0], current[1] - came_from[current][1]))]
            current = came_from[current]

    def neighbors(current):
        valid_neighbors = []
        for coordinate_change in [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]:
            neighbor = (current[0] + coordinate_change[0], current[1] + coordinate_change[1])
            if not (neighbor in obstacles): valid_neighbors.append(neighbor)
        return valid_neighbors

    astar_start_time = time.time()
    
    # g_score[n] is the cheapest path from start to n currently known; f_score[n] = g_score[n] + h(n)
    open_set, came_from, g_scores, f_scores = [], {}, {}, {}
    g_scores[start], f_scores[start] = 0, h(start, goal)

    heapq.heappush(open_set, (f_scores[start], start))

    while open_set:
        if time.time() - astar_start_time > 0.25: return "timeout"
        
        current = heapq.heappop(open_set)[1]
        if current == goal: return reconstruct_path(came_from, current)

        for neighbor in neighbors(current):
            tentative_g_score = g_scores[current] + 1
            if tentative_g_score < g_scores.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_scores[neighbor] = tentative_g_score
                f_scores[neighbor] = tentative_g_score + h(neighbor, goal)
                if neighbor not in open_set:
                    heapq.heappush(open_set, (f_scores[neighbor], neighbor))

    return "none"

def dodge(): pass


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

                    x_wall_coordinates, y_wall_coordinates = [wall[0] for wall in wall_grid_coordinates], [-wall[1] for wall in wall_grid_coordinates]
                    plt.scatter(x_wall_coordinates, y_wall_coordinates, color="black")
                    plt.scatter(self_grid_coordinates[0], -self_grid_coordinates[1], color="blue")
                    plt.scatter(closest_enemy_grid_coordinates[0], -closest_enemy_grid_coordinates[1], color="red")
                    plt.show()

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
