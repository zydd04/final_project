"""
ECSE 211 Final Project - Snow removal robot.

Phase 1 (mapping):
    1. Start at the origin (0, 0), facing +y.
    2. Drive to the tile just outside the street entrance (the s1-s2 edge).
    3. Walk the street one tile at a time, using the ultrasonic sensor to find
       the walls (boxes), until the street ends (we leave the corridor, hit a
       dead end, or reach the map edge).
    4. Retrace the exact path back to the origin.
    5. Save the street map to street_map.json and wait for the facilitator
       before Phase 2.

Usage (the only inputs are the 4 points s1, s2, d1, d2):
    python3 final.py 1,3 1,2 2,-1 3,-1
    python3 final.py                      # prompts for the 4 points

Coordinate conventions:
    - Grid units: intersections are integer points, origin = lower-left
      intersection. Tile (i, j) is the square with lower-left corner (i, j),
      so its centre is (i + 0.5, j + 0.5).
    - Odometry is in cm.
    - Heading: 0 deg = +y, 90 deg = +x, increasing clockwise.
"""

from utils.brick import Motor, EV3UltrasonicSensor, reset_brick, wait_ready_sensors
import json
import math
import re
import sys
import time

# ---------------------------------------------------------------------------
# Hardware configuration (CHECK THESE AGAINST YOUR ROBOT)
# ---------------------------------------------------------------------------
LEFT_MOTOR_PORT = "A"
RIGHT_MOTOR_PORT = "B"
CONVEYOR_MOTOR_PORT = "C"
ULTRASONIC_PORT = 2          # front-facing ultrasonic sensor

# Your drive motors are mounted backwards: a negative dps drives forward.
MOTOR_DIR = -1

# Geometry (cm)
TILE = 30.48
RADIUS = 2.13                # wheel radius
TRACK_WIDTH = 14.5           # distance between the two wheels
US_OFFSET = 8.0              # wheel axle centre -> ultrasonic sensor face (forward)

# Map bounds (in tiles). x intersections in [-1, 8], y in [-1, 4].
MIN_TILE_X, MAX_TILE_X = -1, 7
MIN_TILE_Y, MAX_TILE_Y = -1, 3

START_HEADING = 0.0          # robot is placed at the origin facing +y

# Motion tuning
DRIVE_DPS = 320              # cruise wheel speed
DRIVE_MIN_DPS = 90           # speed near the end of a move
SLOWDOWN_DIST = 8.0          # cm before target where we start slowing down
HEADING_KP = 6.0             # dps of correction per degree of heading error
TURN_MAX_DPS = 280
TURN_MIN_DPS = 45
TURN_KP = 5.0
TURN_TOLERANCE = 1.5         # deg
MOVE_TIMEOUT = 15.0          # s
TURN_TIMEOUT = 6.0           # s
LOOP_DT = 0.01

# Sensing
# From a tile centre, a box in the neighbouring tile is ~TILE/2 - US_OFFSET
# away (~7 cm); an open neighbour gives > 1.5*TILE - US_OFFSET (~38 cm).
WALL_THRESHOLD = TILE - US_OFFSET          # ~22 cm, halfway between the two
US_MAX_RANGE = 255.0
OBSTACLE_STOP_DIST = 10.0    # stop if something is this close while approaching
MAX_POSITION_CORRECTION = 8.0  # ignore wall-based corrections larger than this

# Time budget (phase must finish in 120 s, including the trip home)
PHASE1_LIMIT = 120.0
MAPPING_BUDGET = 75.0        # stop exploring after this and head home

MAP_FILE = "street_map.json"

# Heading -> (dx, dy) in tiles
DIRS = {0: (0, 1), 90: (1, 0), 180: (0, -1), 270: (-1, 0)}

# ---------------------------------------------------------------------------
# Hardware setup
# ---------------------------------------------------------------------------
leftmotor = Motor(LEFT_MOTOR_PORT)
rightmotor = Motor(RIGHT_MOTOR_PORT)
conveyor_motor = Motor(CONVEYOR_MOTOR_PORT)
us_sensor = EV3UltrasonicSensor(ULTRASONIC_PORT)

wait_ready_sensors()

leftmotor.reset_encoder()
rightmotor.reset_encoder()
conveyor_motor.reset_encoder()

# Odometry state
x = 0.0
y = 0.0
theta = START_HEADING
prev_left_encoder = 0
prev_right_encoder = 0

# Every point the robot successfully drove to (cm). Used to retrace the path.
trail = [(0.0, 0.0)]


# ---------------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------------
def set_wheels(left_dps, right_dps):
    """Set wheel speeds where positive = forward for that wheel."""
    leftmotor.set_dps(MOTOR_DIR * left_dps)
    rightmotor.set_dps(MOTOR_DIR * right_dps)


def stop_motors():
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)


def clamp(value, low, high):
    return max(low, min(high, value))


def get_shortest_angle(target_heading, current_heading):
    """Signed turn in [-180, 180) from current to target (positive = clockwise)."""
    return (target_heading - current_heading + 180) % 360 - 180


def update_odometer():
    """Update robot position and heading using wheel encoders."""
    global x, y, theta, prev_left_encoder, prev_right_encoder

    current_left = MOTOR_DIR * leftmotor.get_encoder()
    current_right = MOTOR_DIR * rightmotor.get_encoder()

    d_left = (current_left - prev_left_encoder) / 360.0 * 2 * math.pi * RADIUS
    d_right = (current_right - prev_right_encoder) / 360.0 * 2 * math.pi * RADIUS

    prev_left_encoder = current_left
    prev_right_encoder = current_right

    d_theta = (d_left - d_right) / TRACK_WIDTH      # clockwise positive
    d_dist = (d_left + d_right) / 2.0

    mid_heading = math.radians(theta) + d_theta / 2.0
    x += d_dist * math.sin(mid_heading)
    y += d_dist * math.cos(mid_heading)
    theta = (theta + math.degrees(d_theta)) % 360.0


def print_pose(prefix=""):
    print(f"{prefix}X: {x:7.2f} cm | Y: {y:7.2f} cm | Theta: {theta:6.1f} deg")


def read_distance(samples=5):
    """Median-filtered ultrasonic distance in cm (US_MAX_RANGE if no echo)."""
    values = []
    for _ in range(samples):
        value = us_sensor.get_value()
        if value is not None and value > 0:
            values.append(float(value))
        time.sleep(0.02)
    if not values:
        return US_MAX_RANGE
    values.sort()
    return values[len(values) // 2]


# ---------------------------------------------------------------------------
# Motion
# ---------------------------------------------------------------------------
def turn_to(target_heading):
    """Rotate in place to an absolute heading (shortest direction)."""
    target_heading %= 360.0
    start = time.time()
    try:
        update_odometer()
        delta = get_shortest_angle(target_heading, theta)
        while abs(delta) > TURN_TOLERANCE:
            if time.time() - start > TURN_TIMEOUT:
                print("Turn timed out!")
                break
            speed = clamp(abs(delta) * TURN_KP, TURN_MIN_DPS, TURN_MAX_DPS)
            if delta > 0:          # clockwise
                set_wheels(speed, -speed)
            else:
                set_wheels(-speed, speed)
            time.sleep(LOOP_DT)
            update_odometer()
            delta = get_shortest_angle(target_heading, theta)
    finally:
        stop_motors()
        time.sleep(0.1)
        update_odometer()


def drive_to(target, check_obstacles=False):
    """
    Drive straight to `target` (cm), assuming we already face it.
    Holds the heading toward the target. Returns False if blocked/timed out.
    """
    tx, ty = target
    start_heading = math.radians(theta)
    start = time.time()
    last_us_check = 0.0

    try:
        while True:
            update_odometer()
            dx, dy = tx - x, ty - y
            # Remaining distance measured along the original direction of travel
            along = dx * math.sin(start_heading) + dy * math.cos(start_heading)
            if along <= 0.5:
                return True

            if time.time() - start > MOVE_TIMEOUT:
                print("Move timed out!")
                return False

            if check_obstacles and time.time() - last_us_check > 0.1:
                last_us_check = time.time()
                d = us_sensor.get_value()
                if d is not None and 0 < d < OBSTACLE_STOP_DIST and along > OBSTACLE_STOP_DIST:
                    print(f"Obstacle ahead ({d:.0f} cm)!")
                    return False

            speed = DRIVE_DPS
            if along < SLOWDOWN_DIST:
                speed = max(DRIVE_MIN_DPS, DRIVE_DPS * along / SLOWDOWN_DIST)

            correction = 0.0
            if along > 3.0:   # bearing gets noisy right at the target
                bearing = math.degrees(math.atan2(dx, dy)) % 360
                error = get_shortest_angle(bearing, theta)
                correction = clamp(error * HEADING_KP, -speed / 2, speed / 2)

            set_wheels(speed + correction, speed - correction)
            time.sleep(LOOP_DT)
    finally:
        stop_motors()
        time.sleep(0.05)
        update_odometer()


def go_to(target, check_obstacles=False, record=True):
    """Turn toward and drive to a point in cm."""
    dx, dy = target[0] - x, target[1] - y
    if math.hypot(dx, dy) < 1.0:
        return True
    turn_to(math.degrees(math.atan2(dx, dy)))
    ok = drive_to(target, check_obstacles)
    if ok and record:
        trail.append((float(target[0]), float(target[1])))
    return ok


# ---------------------------------------------------------------------------
# Grid helpers
# ---------------------------------------------------------------------------
def tile_center(tile):
    return ((tile[0] + 0.5) * TILE, (tile[1] + 0.5) * TILE)


def tile_in_bounds(tile):
    return (MIN_TILE_X <= tile[0] <= MAX_TILE_X and
            MIN_TILE_Y <= tile[1] <= MAX_TILE_Y)


def neighbour(tile, heading):
    dx, dy = DIRS[heading]
    return (tile[0] + dx, tile[1] + dy)


def heading_between(a, b):
    for h, (dx, dy) in DIRS.items():
        if (a[0] + dx, a[1] + dy) == tuple(b):
            return h
    raise ValueError(f"Tiles {a} and {b} are not adjacent")


def street_entrance(s1, s2):
    """
    Return (approach_tile, entry_tile, heading_in).
    s1-s2 is a one-tile edge; the street is on one side of it. We approach
    from the side that is closer to the origin (the open side), and verify
    with the sensor once inside.
    """
    (x1, y1), (x2, y2) = s1, s2
    if x1 == x2 and abs(y1 - y2) == 1:          # vertical edge
        j = min(y1, y2)
        a, b = (x1 - 1, j), (x1, j)
    elif y1 == y2 and abs(x1 - x2) == 1:        # horizontal edge
        i = min(x1, x2)
        a, b = (i, y1 - 1), (i, y1)
    else:
        raise ValueError(f"s1={s1} and s2={s2} must be adjacent intersections")

    def dist_to_origin(t):
        cx, cy = tile_center(t)
        return math.hypot(cx, cy)

    candidates = [t for t in (a, b) if tile_in_bounds(t)]
    if len(candidates) == 2:
        approach = min(candidates, key=dist_to_origin)
    else:
        approach = candidates[0]
    entry = b if approach == a else a
    return approach, entry, heading_between(approach, entry)


# ---------------------------------------------------------------------------
# Sensing walls
# ---------------------------------------------------------------------------
def correct_position_from_wall(tile, heading, distance):
    """Use a measured wall distance to fix odometry along that axis."""
    global x, y
    cx, cy = tile_center(tile)
    centre_to_wall = distance + US_OFFSET
    half = TILE / 2.0

    if heading == 0:
        new, old, axis = cy + half - centre_to_wall, y, "y"
    elif heading == 180:
        new, old, axis = cy - half + centre_to_wall, y, "y"
    elif heading == 90:
        new, old, axis = cx + half - centre_to_wall, x, "x"
    else:
        new, old, axis = cx - half + centre_to_wall, x, "x"

    if abs(new - old) <= MAX_POSITION_CORRECTION:
        if axis == "x":
            x = new
        else:
            y = new


def probe(tile, heading):
    """Face `heading` and return True if there is a wall in that neighbour."""
    turn_to(heading)
    distance = read_distance()
    is_wall = distance < WALL_THRESHOLD
    if is_wall:
        correct_position_from_wall(tile, heading, distance)
    names = {0: "+y", 90: "+x", 180: "-y", 270: "-x"}
    print(f"  {names[heading]}: {distance:5.1f} cm -> {'WALL' if is_wall else 'open'}")
    return is_wall


# ---------------------------------------------------------------------------
# Phase 1: mapping
# ---------------------------------------------------------------------------
def travel_to_approach(approach_tile):
    """Go from the current position to the approach tile with an L-shaped path."""
    target = tile_center(approach_tile)
    start = (x, y)

    for first_axis in ("y", "x"):
        corner = (start[0], target[1]) if first_axis == "y" else (target[0], start[1])
        print(f"Approach path ({first_axis} first) via {corner}")
        if go_to(corner, check_obstacles=True) and go_to(target, check_obstacles=True):
            return True
        print("Path blocked, going back to try the other order...")
        go_to(start, record=False)
        # Drop anything recorded for the failed attempt
        while len(trail) > 1 and trail[-1] != start:
            trail.pop()
    return False


def map_street(entry_tile, heading, deadline):
    """
    Walk the street starting at entry_tile (we are on the tile just before it,
    facing `heading`). Returns (street_tiles, exit_tile, wrong_side_flag).

    Each street tile entry: {"tile": (i, j), "walls": {heading: bool}}
    """
    street = []
    visited = set()
    tile = entry_tile

    if not go_to(tile_center(tile)):
        return street, None, False

    while True:
        if time.time() > deadline:
            print("Mapping time budget used up - heading home.")
            return street, None, False

        print(f"\nTile {tile}:")
        visited.add(tile)
        front = heading
        left = (heading - 90) % 360
        right = (heading + 90) % 360
        walls = {}

        walls[front] = probe(tile, front)
        walls[left] = probe(tile, left)
        if not walls[front] and walls[left]:
            # Straight corridor: the street is 1 tile wide, so the right side
            # must be a wall too. Skip probing it to save time.
            walls[right] = True
        else:
            walls[right] = probe(tile, right)

        # Both sides open -> we are no longer inside the corridor.
        if not walls[left] and not walls[right]:
            if not street:
                print("No walls at the first tile: we entered from the wrong side.")
                return street, None, True
            print("Left the street (no side walls).")
            return street, tile, False

        street.append({"tile": tile, "walls": {str(h): w for h, w in walls.items()}})

        # Choose where to go next: straight, else left, else right.
        next_heading = None
        for h in (front, left, right):
            if not walls[h]:
                next_heading = h
                break

        if next_heading is None:
            print("Dead end: end of street.")
            return street, None, False

        next_tile = neighbour(tile, next_heading)
        if not tile_in_bounds(next_tile):
            print("Street reaches the map edge: end of street.")
            return street, None, False
        if next_tile in visited:
            print("Next tile already visited: end of street.")
            return street, None, False

        heading = next_heading
        if not go_to(tile_center(next_tile)):
            print("Could not reach next tile - stopping mapping.")
            return street, None, False
        tile = next_tile


def simplify_path(points):
    """Remove waypoints that lie on a straight line between their neighbours."""
    out = []
    for p in points:
        if out and math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) < 1.0:
            continue
        while len(out) >= 2:
            (ax, ay), (bx, by) = out[-2], out[-1]
            cross = (bx - ax) * (p[1] - ay) - (by - ay) * (p[0] - ax)
            dot = (bx - ax) * (p[0] - bx) + (by - ay) * (p[1] - by)
            seg = math.hypot(bx - ax, by - ay)
            if abs(cross) / seg < 1.0 and dot > 0:   # collinear, same direction
                out.pop()
            else:
                break
        out.append(p)
    return out


def return_to_origin():
    """Retrace every point we visited, in reverse, back to (0, 0)."""
    path = simplify_path(list(reversed(trail)))
    print(f"\nReturning to origin via {len(path)} waypoints")
    for p in path[1:]:
        go_to(p, record=False)
    go_to((0.0, 0.0), record=False)
    turn_to(START_HEADING)
    print_pose("Home: ")


def run_phase1(s1, s2, d1, d2):
    start_time = time.time()
    mapping_deadline = start_time + MAPPING_BUDGET

    approach, entry, heading_in = street_entrance(s1, s2)
    print(f"Street entrance between {s1} and {s2}")
    print(f"Approach tile {approach} -> entry tile {entry}, heading {heading_in}")

    street, exit_tile = [], None
    if travel_to_approach(approach):
        street, exit_tile, wrong_side = map_street(entry, heading_in, mapping_deadline)
        if wrong_side:
            # The tile we came from is actually the first street tile.
            approach, entry = entry, approach
            heading_in = heading_between(approach, entry)
            street, exit_tile, _ = map_street(entry, heading_in, mapping_deadline)
    else:
        print("Could not reach the street entrance.")

    return_to_origin()

    street_map = {
        "s1": s1, "s2": s2, "d1": d1, "d2": d2,
        "approach_tile": approach,
        "entry_tile": entry,
        "entry_heading": heading_in,
        "street": street,                 # ordered from the entrance
        "exit_tile": exit_tile,           # first open tile past the end, or None
        "street_length_tiles": len(street),
    }
    with open(MAP_FILE, "w") as f:
        json.dump(street_map, f, indent=2)

    elapsed = time.time() - start_time
    print(f"\nPhase 1 done in {elapsed:.1f} s "
          f"({'OK' if elapsed <= PHASE1_LIMIT else 'OVER TIME'}).")
    print("Street tiles:", [s["tile"] for s in street])
    return street_map


# ---------------------------------------------------------------------------
# Phase 2: snow removal (TODO)
# ---------------------------------------------------------------------------
def run_phase2(street_map):
    print("Phase 2 not implemented yet.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def parse_args(args):
    """Read the 4 points s1 s2 d1 d2, e.g. '1,3 1,2 2,-1 3,-1'."""
    if not args:
        args = [input(f"{name} (x,y): ") for name in ("s1", "s2", "d1", "d2")]
    nums = [int(round(float(n))) for n in re.findall(r"-?\d+(?:\.\d+)?", " ".join(args))]
    if len(nums) != 8:
        print(__doc__)
        sys.exit("Expected 4 points: s1 s2 d1 d2 (each as x,y)")
    s1, s2, d1, d2 = [tuple(nums[i:i + 2]) for i in range(0, 8, 2)]
    return s1, s2, d1, d2


if __name__ == "__main__":
    s1, s2, d1, d2 = parse_args(sys.argv[1:])
    try:
        conveyor_motor.set_dps(0)
        street_map = run_phase1(s1, s2, d1, d2)
        stop_motors()
        input("\nWaiting for facilitator. Press Enter to start Phase 2...")
        run_phase2(street_map)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        stop_motors()
        conveyor_motor.set_dps(0)
        reset_brick()
