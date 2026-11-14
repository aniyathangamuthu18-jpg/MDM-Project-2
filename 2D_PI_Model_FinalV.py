#2D PI model with obstacle stopping and lane changes.


import csv
import math
from pathlib import Path
import random


# TEAM SOURCE: 1D_Model_ResistiveForces.py (copied parameter blocks)
d_0 =5 #minimum gap between cars
T_h = 2.5 # time headway between cars
v_max = 70 * 0.44704    #speed limit in m/s

pid_freq = 10   #number of pid updates per second
time_step = 1 / pid_freq    #the time duration in seconds between each iteration


# TEAM SOURCE: 1D_Model_ResistiveForces.py (copied resistance constants)
m = 1500
rho = 1.225
C_d = 0.3
A = 2.2
C_rr = 0.015
g = 9.81

# TEAM SOURCE: 2D_PID_2.py (copied optional force limits)
max_engine_force = 4000
max_braking_force = 8000

# EVALUATED 1D PI CONFIGURATION: K_i is 0.05 for the report's PI case.
K_p = 0.4
K_i = 0.05
K_d = 0.0
k_distance = 0.5


# Obstacle and lane-change settings


scenario = "obstacle"  # choose clear, lead, obstacle, off-path, blocked or random
simulation_duration = 60.0  # total time to run the simulation (s)
initial_following_speed = 10.0  # starting speed of the following car (m/s)
random_seed = 7  # use the same number to repeat the same random obstacles
save_csv = True  # save each timestep so we can plot the results later

# controls whether and how the car can change lanes
enable_lane_changes = True  # False makes the car stop instead of going around
lane_change_duration = 4.0  # used to plan the move length; not a fixed actual time (s)
minimum_lane_change_distance = 35.0  # shortest forward distance for the sideways move (m)
maximum_lane_change_speed = 15.0  # lower speed while passing and changing lanes (m/s)
maximum_lateral_acceleration = 2.5  # used to limit speed on the sideways path (m/s^2)
# lane 0 is the starting lane; lanes 1 and 2 are on either side
# these aliases keep the copied teammate lane functions unchanged
min_gap = d_0
time_headway = T_h

# leave these False for the baseline; turn on to compare the extra physics
use_force_limits = False  # cap the driving and braking forces if True
use_actuator_lag = False  # delay the actuator response if True
actuator_time_constant = 0.3  # response time used when actuator lag is on (s)

# Additional assumptions used by the 2D model.
minimum_acceleration_command = -8.0  # lowest requested acceleration (m/s^2)
maximum_acceleration_command = 2.0  # highest requested acceleration (m/s^2)
car_width = 2.0341  # Mean of 10 mirror-inclusive widths, not a UK fleet mean.
car_length = 4.5  # Assumption, metres.
lane_width = 3.65  # selected lane width, not a UK average (m)
lane_positions = {0: 0.0, 1: lane_width, 2: -lane_width}
side_clearance = 0.20  # extra space outside the car body and mirrors (m)
obstacle_detection_range = 150.0  # look this far ahead for relevant objects (m)
obstacle_gap_gain = 0.5  # how strongly clearance changes the desired speed (1/s)
obstacle_spawn_probability = 0.02  # 2% chance per 0.1 s in the random scenario

# checks numbers are valid before running
def all_values_are_finite(values):
    """Check the numbers using a simple loop."""
    for value in values:
        if not math.isfinite(value):
            return False
    return True


def make_pose(x, y, heading):
    """Group a road position and heading in an ordinary dictionary."""
    return {"x": x, "y": y, "heading": heading}

# Heading values are radians measured along the planned path.


# current working road: straight lanes along x, with lane centres given above
# bends and roundabouts will need their completed geometry before they are run


def make_obstacle(x, y, radius, name="obstacle", appears_at=0.0):
    """Create one stationary obstacle as an ordinary dictionary."""
    return {
        "x": x,
        "y": y,
        "radius": radius,
        "name": name,
        "appears_at": appears_at,
    }


def body_clearance(pose, obstacle):
    """Find the space between the car body and an obstacle. Negative means a collision."""
    difference_x, difference_y = obstacle["x"] - pose["x"], obstacle["y"] - pose["y"]
    forward = difference_x * math.cos(pose["heading"]) + difference_y * math.sin(
        pose["heading"]
    )
    lateral = -difference_x * math.sin(pose["heading"]) + difference_y * math.cos(
        pose["heading"]
    )
    return (
        math.hypot(
            max(abs(forward) - car_length / 2, 0.0),
            max(abs(lateral) - car_width / 2, 0.0),
        )
        - obstacle["radius"]
    )


def command_limits():
    minimum_command, maximum_command = (
        minimum_acceleration_command,
        maximum_acceleration_command,
    )
    if use_force_limits:
        minimum_command = max(minimum_command, -max_braking_force / m)
        maximum_command = min(maximum_command, max_engine_force / m)
    return minimum_command, maximum_command


def vehicle_forces(actual_acceleration, following_car_v):
    """Calculate the forces using the team's equations."""
    # use the familiar variable names from the earlier 2D force code
    mass = m
    air_density = rho
    drag_coeff = C_d
    front_area = A
    friction_factor = C_rr

    # TEAM SOURCE: 2D_PID_2.py, copied driving and braking equations.
    engine_force = mass * max(actual_acceleration, 0) # calculates engine force with F = ma, max function ensures engine force has positive acceleration
    braking_force = mass * max(-actual_acceleration, 0)  #calculate braking force with F = ma, max function ensures braking force is calculated when acceleration is negative

    # NEW connection: the agreed model makes force caps optional.
    if use_force_limits:
        # TEAM SOURCE: 2D_PID_2.py, copied force-limit equations.
        engine_force = min(engine_force, max_engine_force) # sets limit of max engine force average car can produce
        braking_force = min(braking_force, max_braking_force) #sets limit of braking force

    # TEAM SOURCE: 2D_PID_2.py, copied aerodynamic drag equation.
    drag_force = 0.5 * air_density * drag_coeff * front_area * following_car_v**2 # calcuate air drag force

    # ADAPTED from the team's rolling-force equation: use configurable gravity
    # instead of a hard-coded 9.81. The default remains exactly 9.81 m/s^2.
    friction_force = friction_factor * mass * g

    # TEAM SOURCE: 2D_PID_2.py, copied net-force equation.
    net_force = (engine_force - braking_force - drag_force - friction_force) # combines forces
    return engine_force, braking_force, net_force


# TEAM SOURCE: 2D_model_lane_change.py. The three functions below are copied
# verbatim (line endings normalised). The independent teammate loops are NOT run.
# lane_change_decision is an initial gap check; extra prediction checks follow.
def find_nearest_vehicles(current_x, current_lane, vehicles):

    front_vehicle = None
    rear_vehicle = None

    front_gap = float("inf")
    rear_gap = float("inf")

    for vehicle in vehicles:

        if vehicle["lane"] == current_lane:
            distance = vehicle["x"] - current_x

            if distance > 0 and distance < front_gap:

                front_gap = distance
                front_vehicle = vehicle

            elif distance < 0 and abs(distance)< rear_gap:

                rear_gap = abs(distance)
                rear_vehicle = vehicle

    return front_vehicle, rear_vehicle, front_gap, rear_gap

def lane_change_decision(current_x, current_v, target_lane, vehicles):

    front, rear, front_gap, rear_gap = find_nearest_vehicles(
        current_x, target_lane, vehicles
    )

    safe_front_gap = min_gap + time_headway * current_v

    if rear is not None:

        safe_rear_gap = min_gap + time_headway * rear["v"]

    else:

        safe_rear_gap = min_gap

    if front_gap > safe_front_gap and rear_gap > safe_rear_gap:

        return True

    else:
        
        return False


def calculate_lane_position(start_y, target_y, elapsed_time, duration):

    progress = min(elapsed_time / duration, 1) 

    smooth_progress = 3 * progress**2 - 2 * progress**3

    new_y= start_y + ( target_y - start_y) * smooth_progress

    return new_y


# NEW: use the teammate's smooth interpolation along forward road distance.
# This prevents the car sliding sideways while stopped. The original function
# uses time/duration names, but only their dimensionless ratio is required.
def lane_path(x, plan, lane_y):
    if plan is None:
        return lane_y, 0.0, 0.0
    progress = (x - plan["start_x"]) / plan["length"]
    if progress <= 0:
        progress = 0.0
    if progress >= 1:
        return plan["target_y"], 0.0, 0.0
    lateral_position = calculate_lane_position(
        plan["start_y"], plan["target_y"], progress, 1.0
    )
    change = plan["target_y"] - plan["start_y"]
    slope = change * (6 * progress - 6 * progress**2) / plan["length"]
    second_derivative = change * (6 - 12 * progress) / plan["length"]**2
    return lateral_position, slope, second_derivative


def car_pose(x, plan, lane_y):
    y, slope, second_derivative = lane_path(x, plan, lane_y)
    return make_pose(x, y, math.atan(slope))


def body_half_width(pose):
    # Rotating a rectangle can make its sideways extent greater than half width.
    return (car_width / 2 * abs(math.cos(pose["heading"]))
            + car_length / 2 * abs(math.sin(pose["heading"])))


def road_boundaries():
    centres = list(lane_positions.values())
    return min(centres) - lane_width / 2, max(centres) + lane_width / 2


def inside_road(pose, margin=0.0):
    left_edge, right_edge = road_boundaries()
    half_width = body_half_width(pose)
    return (pose["y"] - half_width - margin >= left_edge
            and pose["y"] + half_width + margin <= right_edge)


def vehicles_overlap(ego_pose, traffic_x, traffic_y, margin=0.0):
    # Separating-axis test: rotated ego rectangle against aligned traffic car.
    heading = ego_pose["heading"]
    for angle in [heading, heading + math.pi / 2, 0.0, math.pi / 2]:
        separation = abs((traffic_x - ego_pose["x"]) * math.cos(angle)
                         + (traffic_y - ego_pose["y"]) * math.sin(angle))
        ego_extent = ((car_length / 2 + margin) * abs(math.cos(heading - angle))
                      + (car_width / 2 + margin) * abs(math.sin(heading - angle)))
        traffic_extent = (car_length / 2 * abs(math.cos(angle))
                          + car_width / 2 * abs(math.sin(angle)))
        if separation > ego_extent + traffic_extent:
            return False
    return True


def traffic_at_time(traffic, time):
    vehicles = []
    for vehicle in traffic:
        vehicles.append({"x": vehicle["x"] + vehicle["v"] * time,
                         "v": vehicle["v"], "lane": vehicle["lane"]})
    return vehicles


def candidate_plan(x, speed, lane_y, target_lane, returning):
    length = max(minimum_lane_change_distance, speed * lane_change_duration)
    # Conservative bound: |curvature| <= 6*|change|/length^2 for this path.
    curvature_bound = 6 * abs(lane_positions[target_lane] - lane_y) / length**2
    speed_cap = maximum_lane_change_speed
    if curvature_bound > 0:
        speed_cap = min(speed_cap, math.sqrt(maximum_lateral_acceleration / curvature_bound))
    return {"start_x": x, "start_y": lane_y, "target_y": lane_positions[target_lane],
            "target_lane": target_lane, "length": length, "speed_cap": speed_cap,
            "returning": returning}


def manoeuvre_is_clear(plan, x, speed, lane_y, traffic, obstacles, time):
    """Check the planned move before and during a lane change.

    Check the whole car fits inside the road and misses known obstacles.
    Check future traffic against a range of possible forward positions.
    These sampled checks are a model approximation, not a safety guarantee.
    """
    end_x = plan["start_x"] + plan["length"]
    # The candidate must stay clear beyond the lateral move, through the pass.
    end_x = max(end_x, plan.get("clear_until", end_x))
    remaining = end_x - x
    if remaining <= 0:
        return True
    if speed > plan["speed_cap"] + 0.01:
        return False  # Slow first; do not start an over-fast lateral manoeuvre.
    margin = side_clearance + 0.05
    minimum_command, maximum_command = command_limits()
    acceleration_bound = max(maximum_command, 0.01)
    duration_bound = 2 * remaining / max(speed, 1.0) + 2.0
    for sample in range(101):
        progress = sample / 100
        sample_x = x + progress * remaining
        pose = car_pose(sample_x, plan, lane_y)
        if not inside_road(pose, margin):
            return False
        for obstacle in obstacles:
            if obstacle["appears_at"] <= time and body_clearance(pose, obstacle) <= margin:
                return False
        # Traffic clearance checked over an envelope in longitudinal position.
        # At any time up to duration_bound, retain headway to front/rear traffic
        # whenever its lane overlaps any part of this lateral manoeuvre.
        elapsed = progress * duration_bound
        forward_max = min(remaining, speed * elapsed + 0.5 * acceleration_bound * elapsed**2)
        forward_min = 0.0  # Includes stopping during the manoeuvre.
        for vehicle in traffic:
            traffic_y = lane_positions[vehicle["lane"]]
            path_low = min(lane_y, plan["target_y"]) - car_width - margin
            path_high = max(lane_y, plan["target_y"]) + car_width + margin
            if not path_low <= traffic_y <= path_high:
                continue
            vehicle_x = vehicle["x"] + vehicle["v"] * (time + elapsed)
            front_buffer = car_length + d_0 + T_h * speed
            rear_buffer = car_length + d_0 + T_h * vehicle["v"]
            if x + forward_min - rear_buffer <= vehicle_x <= x + forward_max + front_buffer:
                return False
    return True


# checks the settings before running the car simulation
def check_inputs():
    values = [simulation_duration, initial_following_speed, time_step, K_p, K_i,
              K_d, d_0, T_h, m, rho, C_d, A, C_rr, g, v_max,
              minimum_acceleration_command, maximum_acceleration_command,
              car_width, car_length, lane_width, side_clearance,
              obstacle_detection_range, obstacle_gap_gain,
              obstacle_spawn_probability, max_engine_force, max_braking_force,
              actuator_time_constant, lane_change_duration,
              minimum_lane_change_distance, maximum_lane_change_speed,
              maximum_lateral_acceleration]
    if not all_values_are_finite(values):
        raise ValueError("The settings must be valid numbers, not infinity or NaN.")
    if simulation_duration <= 0 or initial_following_speed < 0:
        raise ValueError("Use a positive duration and a non-negative starting speed.")
    if time_step <= 0 or m <= 0 or actuator_time_constant <= 0 or K_d != 0:
        raise ValueError("Timestep, mass and response time must be positive. PI uses K_d=0.")
    if not 0 <= obstacle_spawn_probability <= 1:
        raise ValueError("Obstacle probability must be between zero and one.")
    if min(car_length, car_width, lane_width, obstacle_detection_range,
           max_engine_force, max_braking_force, lane_change_duration,
           minimum_lane_change_distance, maximum_lane_change_speed,
           maximum_lateral_acceleration) <= 0:
        raise ValueError("Dimensions, ranges and physical limits must be positive.")
    if minimum_acceleration_command >= 0 or maximum_acceleration_command <= 0:
        raise ValueError("The acceleration limits must include negative and positive values.")
    if min(T_h, d_0, side_clearance, v_max) < 0:
        raise ValueError("Headway, gap, side clearance and speed limit cannot be negative.")


# runs one car simulation using the variables defined at the top
# ============================================================================
# 3. ROAD MORPHOLOGY FUNCTIONS - ADD TEAMMATE WORK HERE LATER
# ============================================================================
# BENDS: add completed geometry and speed-planning functions here.
# ROUNDABOUTS: add completed geometry, route/lap tracking and speed planning here.
# Their speed calculations must be called in the ROAD SPEED GAP inside the loop.
# Keep teammate originals unchanged. Do not paste another car simulation loop.
# Current lane geometry and collision checks support straight roads only.


# ADD COMPLETED BEND FUNCTIONS HERE


# ADD COMPLETED ROUNDABOUT FUNCTIONS HERE


# ============================================================================
# CHOOSE THE SCENARIO
# ============================================================================
# choose the objects and traffic for this demonstration
obstacles = []
traffic = []
random_objects = False
if scenario == "obstacle":
    obstacles.append(make_obstacle(100.0, 0.0, 0.8))
elif scenario == "off-path":
    obstacles.append(make_obstacle(100.0, 4.0, 0.8))
elif scenario == "lead":
    traffic.append({"x": 50.0, "v": 20.0, "lane": 0})
elif scenario == "blocked":
    obstacles.append(make_obstacle(100.0, 0.0, 0.8))
    traffic.append({"x": 100.0, "v": 0.0, "lane": 1})
    traffic.append({"x": 100.0, "v": 0.0, "lane": 2})
elif scenario == "random":
    random_objects = True
elif scenario != "clear":
    raise ValueError("Choose clear, lead, obstacle, off-path, blocked, or random.")

# ============================================================================
# START THE SIMULATION - ONE TIMESTEP LOOP
# ============================================================================
check_inputs()
objects = []
for obstacle in obstacles or []:
    if obstacle["radius"] <= 0 or obstacle["appears_at"] < 0:
        raise ValueError("Obstacle radius must be positive and appearance non-negative.")
    if not all_values_are_finite([obstacle["x"], obstacle["y"], obstacle["radius"], obstacle["appears_at"]]):
        raise ValueError("Obstacle geometry must be finite.")
    objects.append(obstacle.copy())
vehicles = []
for vehicle in traffic or []:
    if (vehicle["lane"] not in lane_positions or vehicle["v"] < 0
            or not all_values_are_finite([vehicle["x"], vehicle["v"]])):
        raise ValueError("Traffic needs a valid lane and non-negative prescribed speed.")
    vehicles.append(vehicle.copy())
generator = random.Random(random_seed)
x = 0.0
speed = initial_following_speed
time = 0.0
current_lane = 0
original_lane = 0
lane_y = lane_positions[current_lane]
plan = None
lane_state = "FOLLOWING"
completed_changes = 0
integral_error = 0.0
lagged_command = 0.0
previous_target = None
history = []
minimum_obstacle_clearance = math.inf
status = "completed"
lower, upper = command_limits()

# update the car once per timestep, like the earlier team simulations
number_of_steps = math.ceil(simulation_duration / time_step)
for step in range(number_of_steps):
    if time >= simulation_duration - 1e-10:
        break
    step_duration = min(time_step, simulation_duration - time)
    active_objects = []
    if random_objects and generator.random() < 1 - (1 - obstacle_spawn_probability)**(step_duration / 0.1):
        kind, radius = generator.choice([
            ("small_debris", 0.4), ("large_debris", 0.8),
            ("stopped_vehicle", math.hypot(car_length / 2, car_width / 2))])
        object_x = x + generator.uniform(160, 200)
        object_y = lane_y + generator.choice([-4.0, 0.0, 4.0])
        objects.append(make_obstacle(object_x, object_y, radius,
                                     kind + "_" + str(len(objects)), time))
    for obstacle in objects:
        if obstacle["appears_at"] <= time:
            active_objects.append(obstacle)
    current_traffic = traffic_at_time(vehicles, time)

    # 1. check whether an obstacle blocks the current or planned path
    blocked = False
    for obstacle in active_objects:
        distance_ahead = obstacle["x"] - x
        expected_y, slope, second_derivative = lane_path(obstacle["x"], plan, lane_y)
        if (0 <= distance_ahead <= obstacle_detection_range
                and abs(obstacle["y"] - expected_y) <= car_width / 2 + side_clearance + obstacle["radius"]):
            blocked = True

    # 2. check neighbouring traffic and the full path before changing lanes
    if enable_lane_changes and plan is None:
        possible_lanes = []
        returning = current_lane != original_lane
        if returning:
            # Do not return until the rear has passed original-lane objects.
            return_path_blocked = False
            for obstacle in active_objects:
                overlaps_home = abs(obstacle["y"] - lane_positions[original_lane]) <= (
                    car_width / 2 + side_clearance + obstacle["radius"])
                not_passed = x <= obstacle["x"] + obstacle["radius"] + car_length / 2 + d_0
                nearby = obstacle["x"] - x <= obstacle_detection_range
                if overlaps_home and not_passed and nearby:
                    return_path_blocked = True
            if not return_path_blocked:
                possible_lanes.append(original_lane)
        elif blocked:
            for lane in lane_positions:
                if lane != current_lane and abs(lane_positions[lane] - lane_y) <= lane_width + 0.01:
                    possible_lanes.append(lane)
        for target_lane in possible_lanes:
            quick_check = lane_change_decision(x, speed, target_lane, current_traffic)
            proposed = candidate_plan(x, speed, lane_y, target_lane, returning)
            if not returning:
                clear_until = proposed["start_x"] + proposed["length"]
                for obstacle in active_objects:
                    overlaps_current = abs(obstacle["y"] - lane_y) <= (
                        car_width / 2 + side_clearance + obstacle["radius"])
                    if overlaps_current and 0 <= obstacle["x"] - x <= obstacle_detection_range:
                        clear_until = max(clear_until, obstacle["x"] + obstacle["radius"]
                                          + car_length + d_0)
                proposed["clear_until"] = clear_until
            if quick_check and manoeuvre_is_clear(proposed, x, speed, lane_y,
                                                  vehicles, active_objects, time):
                plan = proposed
                lane_state = "CHANGING_BACK" if returning else "CHANGING_LANE"
                break

    # 3. choose the lowest speed target: speed limit, traffic, objects or road
    desired_gap = d_0 + T_h * speed
    candidates = [(v_max, "free", math.inf, False)]
    # ROAD SPEED GAP: add the teammate's lower bend/roundabout target here.
    # For now there is no extra restriction; v_max still caps the final target.
    road_limit = math.inf
    if math.isnan(road_limit) or road_limit < 0:
        raise ValueError("Road target must be non-negative.")
    candidates.append((road_limit, "road", math.inf, False))
    if plan is not None:
        candidates.append((plan["speed_cap"], "manoeuvre", math.inf, False))
    elif current_lane != original_lane:
        # Keep a manoeuvring speed until a checked return is completed.
        candidates.append((maximum_lane_change_speed, "passing", math.inf, False))
    elif blocked and enable_lane_changes:
        candidates.append((maximum_lane_change_speed, "prepare_manoeuvre", math.inf, False))
    threats = []
    for index, vehicle in enumerate(current_traffic):
        distance_ahead = vehicle["x"] - x
        path_y, slope, second_derivative = lane_path(vehicle["x"], plan, lane_y)
        if (0 <= distance_ahead <= obstacle_detection_range
                and abs(lane_positions[vehicle["lane"]] - path_y) <= car_width + side_clearance):
            gap = distance_ahead - car_length
            target = max(0.0, vehicle["v"] + k_distance * (gap - desired_gap))
            candidates.append((target, "traffic_" + str(index), gap, vehicle["v"] == 0))
            threats.append((gap, speed - vehicle["v"]))
    for index, obstacle in enumerate(active_objects):
        distance_ahead = obstacle["x"] - x
        path_y, slope, second_derivative = lane_path(obstacle["x"], plan, lane_y)
        if (0 <= distance_ahead <= obstacle_detection_range
                and abs(obstacle["y"] - path_y) <= car_width / 2 + side_clearance + obstacle["radius"]):
            gap = distance_ahead - car_length / 2 - obstacle["radius"]
            target = max(0.0, obstacle_gap_gain * (gap - desired_gap))
            candidates.append((target, "object_" + str(index), gap, True))
            threats.append((gap, speed))

    # keep checking the remaining path while the car changes lanes
    if plan is not None and not manoeuvre_is_clear(plan, x, speed, lane_y,
                                                    vehicles, active_objects, time):
        candidates.append((0.0, "manoeuvre_blocked", math.inf, False))
    chosen = candidates[0]
    for candidate in candidates:
        if candidate[0] < chosen[0]:
            chosen = candidate
    target, source, gap, stationary = chosen
    hold = stationary and gap <= d_0 + 0.25 and speed < 0.05
    if hold:
        target = 0.0
        integral_error = 0.0
    if source != previous_target:
        integral_error = 0.0

    # 4. PI controller: turn speed error into an acceleration request
    target_speed = target
    following_car_v = speed


    # TEAM SOURCE: 2D_PID_2.py, copied error and P/I statements.
    error = target_speed - following_car_v # speed error
    P = K_p * error # proportional term
    possible_integral = integral_error + error * step_duration
    possible_command = P + K_i * possible_integral
    close_to_target = abs(error) <= 1.0
    unwinding = error * integral_error < 0
    saturated_high = possible_command >= upper and error > 0
    saturated_low = possible_command <= lower and error < 0
    if (close_to_target or unwinding) and not saturated_high and not saturated_low and not hold:
        integral_error = possible_integral
    I = K_i * integral_error
    command = max(lower, min(upper, P + I))
    emergency = False
    for gap, closing_speed in threats:
        if speed > 0.1 and (gap <= d_0 or
                            (closing_speed > 0 and gap / closing_speed < 1.0)):
            emergency = True
    if emergency:
        command = lower
        integral_error = 0.0
        I = 0.0
    previous_target = source

    # 5. calculate the actual acceleration and update the car's motion
    if use_actuator_lag:
        lagged_command += (1 - math.exp(-step_duration / actuator_time_constant)) * (command - lagged_command)
    else:
        lagged_command = command
    drive, brake, net_force = vehicle_forces(lagged_command, speed)
    if speed <= 0 and net_force <= 0:
        net_force = 0.0
    acceleration = net_force / m
    move_time = step_duration
    if acceleration < 0 and speed + acceleration * step_duration < 0:
        move_time = -speed / acceleration
    distance_travelled = max(0.0, speed * move_time + 0.5 * acceleration * move_time**2)
    new_speed = max(0.0, speed + acceleration * step_duration)

    # speed is along the sideways path too, not just along x
    # use the direction halfway through the move to estimate forward distance
    old_pose = car_pose(x, plan, lane_y)
    midpoint = x + 0.5 * distance_travelled * math.cos(old_pose["heading"])
    midpoint_pose = car_pose(midpoint, plan, lane_y)
    forward_distance = distance_travelled * math.cos(midpoint_pose["heading"])
    new_x = x + forward_distance
    new_pose = car_pose(new_x, plan, lane_y)

    # 6. check the whole car for collisions during this timestep
    samples = max(1, math.ceil(distance_travelled / 0.025))
    for sample in range(samples + 1):
        fraction = sample / samples
        sample_pose = car_pose(x + fraction * forward_distance, plan, lane_y)
        sample_time = time + fraction * step_duration
        if not inside_road(sample_pose):
            status = "road_boundary_violation"
        for obstacle in active_objects:
            clearance = body_clearance(sample_pose, obstacle)
            minimum_obstacle_clearance = min(minimum_obstacle_clearance, clearance)
            if clearance <= 0:
                status = "obstacle_collision"
        for vehicle in vehicles:
            traffic_x = vehicle["x"] + vehicle["v"] * sample_time
            traffic_y = lane_positions[vehicle["lane"]]
            if vehicles_overlap(sample_pose, traffic_x, traffic_y):
                status = "traffic_collision"
            traffic_pose = make_pose(traffic_x, traffic_y, 0.0)
            for obstacle in active_objects:
                if body_clearance(traffic_pose, obstacle) <= 0:
                    status = "invalid_prescribed_traffic_trajectory"
    if plan is not None and new_x >= plan["start_x"] + plan["length"]:
        current_lane = plan["target_lane"]
        lane_y = plan["target_y"]
        was_returning = plan["returning"]
        plan = None
        completed_changes += 1
        lane_state = "FOLLOWING" if was_returning else "PASSING_OBSTACLE"
    curvature = 0.0
    if plan is not None:
        y, slope, second_derivative = lane_path(new_x, plan, lane_y)
        curvature = abs(second_derivative) / (1 + slope**2)**1.5
    history.append({"time_s": time + step_duration, "x_m": new_x, "y_m": new_pose["y"],
                    "speed_m_s": new_speed, "target_m_s": target, "mode": source,
                    "lane": current_lane, "lane_state": lane_state,
                    "command_m_s2": command, "acceleration_m_s2": (new_speed - speed) / step_duration,
                    "drive_force_N": drive, "brake_force_N": brake,
                    "proportional_m_s2": P, "integral_m_s2": I,
                    "lateral_acceleration_m_s2": new_speed**2 * curvature,
                    "emergency": emergency})
    x = new_x
    speed = new_speed
    time += step_duration
    if status != "completed":
        break
clearance = None
if math.isfinite(minimum_obstacle_clearance):
    clearance = minimum_obstacle_clearance
result = {"status": status, "history": history, "final_speed_m_s": speed,
        "final_x_m": x, "final_lane": current_lane, "lane_changes": completed_changes,
        "minimum_obstacle_clearance_m": clearance}


# save the raw trace and print a few checks
if save_csv:
    folder = Path(__file__).resolve().parent / "outputs"
    folder.mkdir(exist_ok=True)
    filename = scenario
    if use_force_limits:
        filename += "_forces"
    if use_actuator_lag:
        filename += "_lag"
    filename += "_trace.csv"
    with (folder / filename).open("w", newline="") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=result["history"][0])
        writer.writeheader()
        writer.writerows(result["history"])
    print("CSV saved to:", folder / filename)
print("Status:", result["status"])
print("Final speed (m/s):", result["final_speed_m_s"])
print("Completed lane changes:", result["lane_changes"])
print("Final group-model results are still pending.")


# ============================================================================
# 4. FINAL 2D RESULTS - ADD YOUR PLOTS AND COMPARISONS HERE LATER
# ============================================================================
# history contains one record per timestep. The checks printed above are not
# the final integrated-model results. Do not copy earlier full-PID results here.

