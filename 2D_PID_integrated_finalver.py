import math
import random

# UPDATED: reproducible demonstration; change seed for another scenario.
random.seed(7)

# e(t) = lead_car_v(t) - following_car_v(t)
# p(t) = K_p * e(t)
# i(t) = K_i * integral e(t)
# d(t) = K_d  * derivative e(t)
# following_car_a = p(t) + i(t) + d(t)
'''
This model takes into account the forces acting onto the following car while driving on a straight road and how they can affect acceleration 
Also has some basic steering mechanics
'''

mass = 1500                 #constants used to calculate forces acting on average car
max_engine_force = 4000
max_braking_force = 8000

drag_coeff = 0.3
front_area = 2.2
air_density = 1.225
friction_factor = 0.015

max_acceleration = 3
max_deceleration = 8

#parameters used to calculate turning physics
wheelbase = 2.7 #distance between back and front wheels
k_steering = 1.5

lead_car_v = 20 # speed
lead_car_x = 50 # x position
lead_car_y = 0 # y position
following_car_v = 10 # speed
following_car_x = 0 #  x position
following_car_y = 0 # y position
following_car_heading = 0 #direction car faces
min_gap = 5 # minimum gap when both cars are at a standstill - whatever speed they are travelling at, will always be at least a 5m gap for safety
time_headway = 2.5 # time taken for the following car at the current speed to reach the lead car
K_gap = 0.1 # controls how strongly the following car responds to the distance between two cars

t = 0.1 # time
K_p = 0.5 # proportional controller - responds to current speed error
K_i = 0.05 # integral controller - accumulates speed error over time
K_d = 0.5 # derivative controller - responds to how quickly error changes
integral_error = 0 # initially set to zero 
previous_error = lead_car_v - following_car_v

lead_x_positions = []
lead_y_positions = []

following_x_positions = []
following_y_positions = []


# UPDATED OBSTACLE SETTINGS (metres). Other vehicle-force parameters retained.
# Width is the mean of 10 mirror-inclusive widths listed by Churchill/Aviva,
# NOT a UK fleet average. Lane width is a selected CD127 dimension, NOT a mean.
car_width = 2.0341
car_length = 4.5
lane_width = 3.65
side_clearance = 0.20
speed_limit = 70 * 0.44704
obstacle_detection_range = 150.0
obstacle_spawn_probability = 0.02  # per existing 0.1-second timestep
obstacle_types = [
    ("small_debris", 0.4),
    ("large_debris", 0.8),
    ("stopped_vehicle", math.hypot(car_length / 2, car_width / 2)),
]
# Run this demonstration on a straight road without scripted lead traffic.
# Set True to test car-following; its prescribed lead does not avoid obstacles.
lead_active = False
random_obstacles = True
obstacles = []
K_obstacle = 0.5
collision_detected = False
minimum_clearance = float("inf")
previous_target_id = None


def relative_position(x, y):
    """Object coordinates resolved along/across the current car heading."""
    dx = x - following_car_x
    dy = y - following_car_y
    forward = dx * math.cos(following_car_heading) + dy * math.sin(following_car_heading)
    sideways = -dx * math.sin(following_car_heading) + dy * math.cos(following_car_heading)
    return forward, sideways


# Actuator parameters, adding small delay 
tau_actuator = 0.3
actual_acceleration = 0.0


for step in range(600):

    dx = lead_car_x - following_car_x
    dy = lead_car_y - following_car_y

    gap = math.sqrt(dx**2 + dy**2) # calculate current gap

    safe_gap = min_gap + (time_headway * following_car_v) # calculate desired safe gap

    gap_error = gap - safe_gap

    # UPDATED: one speed PID receives the most restrictive relevant target.
    target_speed = speed_limit
    target_id = "free"
    relevant_gaps = []
    lead_forward, lead_sideways = relative_position(lead_car_x, lead_car_y)
    lead_relevant = (
        lead_active and 0 < lead_forward <= obstacle_detection_range
        and abs(lead_sideways) <= car_width + side_clearance
    )
    if lead_relevant:
        gap = lead_forward - car_length  # bumper clearance for equal-length cars
        gap_error = gap - safe_gap
        lead_target = max(0.0, lead_car_v + K_gap * gap_error)
        if lead_target < target_speed:
            target_speed, target_id = lead_target, "lead"
        relevant_gaps.append((gap, following_car_v - lead_car_v))

    # UPDATED: normal objects appear far enough away to enter the sensing range.
    # +/-4 m means neighbouring-lane/off-path, not inside the ego lane.
    if random_obstacles and random.random() < obstacle_spawn_probability:
        kind, radius = random.choice(obstacle_types)
        forward = random.uniform(160.0, 200.0)
        sideways = random.choice([-4.0, 0.0, 4.0])
        obstacles.append({
            "type": kind, "radius": radius,
            "x": following_car_x + forward * math.cos(following_car_heading)
                 - sideways * math.sin(following_car_heading),
            "y": following_car_y + forward * math.sin(following_car_heading)
                 + sideways * math.cos(following_car_heading),
        })

    for index, obstacle in enumerate(obstacles):
        forward, sideways = relative_position(obstacle["x"], obstacle["y"])
        if (0 <= forward <= obstacle_detection_range
                and abs(sideways) <= car_width / 2 + side_clearance + obstacle["radius"]):
            obstacle_gap = forward - car_length / 2 - obstacle["radius"]
            # Treat an object as a zero-speed lead: no SUVAT target formula.
            obstacle_target = max(0.0, K_obstacle * (obstacle_gap - safe_gap))
            if obstacle_target < target_speed:
                target_speed, target_id = obstacle_target, ("obstacle", index)
            relevant_gaps.append((obstacle_gap, following_car_v))
            if obstacle_gap <= min_gap + 0.25 and following_car_v < 0.05:
                target_speed, target_id = 0.0, ("obstacle", index)

    # UPDATED: reset controller history on a change of controlling object.
    if target_id != previous_target_id:
        integral_error = 0.0
        previous_error = target_speed - following_car_v
    previous_target_id = target_id
    emergency = any(
        following_car_v > 0.1 and
        (clearance <= min_gap or (closing > 0 and clearance / closing < 1.0))
        for clearance, closing in relevant_gaps
    )

    error = target_speed - following_car_v # speed error
    P = K_p * error # proportional term

    # UPDATED conditional integration: reduce windup under force saturation.
    trial = P + K_i * (integral_error + error * t) + K_d * (error - previous_error) / t
    upper = min(max_acceleration, max_engine_force / mass)
    lower = -min(max_deceleration, max_braking_force / mass)
    if (abs(error) <= 1.0 or error * integral_error < 0) and not (
        (trial >= upper and error > 0) or (trial <= lower and error < 0)
    ):
        integral_error += error * t
    I = K_i * integral_error

    D = K_d * (error - previous_error) / t # derivative term

    acceleration = P + I + D # total PID acceleration

    previous_error = error # save error for next time step

    old_following_car_v = following_car_v

    desired_acceleration = max(-max_deceleration, min(acceleration, max_acceleration)) # sets limits on acceleration to what an average car can do
    # UPDATED: respect force limits in the command and anti-windup.
    desired_acceleration = max(lower, min(desired_acceleration, upper))
    if emergency:
        desired_acceleration = lower
        integral_error = 0.0
    actual_acceleration += (t / tau_actuator) * (desired_acceleration - actual_acceleration) # adds a small delay to the acceleration to simulate actuator delay

    engine_force = mass * max(actual_acceleration, 0) # calculates engine force with F = ma, max function ensures engine force has positive acceleration
    engine_force = min(engine_force, max_engine_force) # sets limit of max engine force average car can produce

    braking_force = mass * max(-actual_acceleration, 0)  #calculate braking force with F = ma, max function ensures braking force is calculated when acceleration is negative
    braking_force = min(braking_force, max_braking_force) #sets limit of braking force

    drag_force = 0.5 * air_density * drag_coeff * front_area * following_car_v**2 # calcuate air drag force
    
    friction_force = friction_factor * mass * 9.81 # calculates friction force between tires and ground

    net_force = (engine_force - braking_force - drag_force - friction_force) # combines forces

    # UPDATED: resistance cannot reverse a stationary vehicle.
    if old_following_car_v <= 0 and net_force <= 0:
        net_force = 0.0
    physical_acceleration = net_force / mass
    move_time = t
    if physical_acceleration < 0 and old_following_car_v + physical_acceleration * t < 0:
        move_time = -old_following_car_v / physical_acceleration
    distance_travelled = max(0.0, old_following_car_v * move_time
                             + 0.5 * physical_acceleration * move_time**2)
    following_car_v = max(0.0, old_following_car_v + physical_acceleration * t)

    # RETAINED simplified bicycle steering, now only towards relevant lead traffic.
    # Straight-road obstacle demonstration holds heading; no obstacle swerving.
    angular_velocity = 0.0
    if lead_relevant:
        desired_heading = math.atan2(dy, dx)
        heading_error = math.atan2(math.sin(desired_heading - following_car_heading),
                                   math.cos(desired_heading - following_car_heading))
        steering_angle = max(-0.5, min(0.5, k_steering * heading_error))
        angular_velocity = following_car_v / wheelbase * math.tan(steering_angle)

    lead_car_x = lead_car_x + (lead_car_v * t) # update lead car position currently just have lead car driving straight

    old_following_heading = following_car_heading
    following_car_heading = following_car_heading + angular_velocity * t #updates following car heading after timestep

    average_heading = (old_following_heading + following_car_heading) / 2
    # distance_travelled already handles a stop occurring within this update.
    old_x, old_y = following_car_x, following_car_y

    following_car_x += (distance_travelled * math.cos(average_heading))
    following_car_y += (distance_travelled * math.sin(average_heading)) #updates position after timestep

    # UPDATED collision reporting: sample swept motion, with extra allowance
    # between samples. This detects failures; it does not guarantee avoidance.
    samples = max(1, math.ceil(distance_travelled / 0.05))
    motion_margin = (distance_travelled + math.hypot(car_length / 2, car_width / 2)
                     * abs(angular_velocity * t)) / (2 * samples)
    for obstacle in obstacles:
        for j in range(samples + 1):
            fraction = j / samples
            px = old_x + fraction * (following_car_x - old_x)
            py = old_y + fraction * (following_car_y - old_y)
            heading = old_following_heading + fraction * angular_velocity * t
            ox, oy = obstacle["x"] - px, obstacle["y"] - py
            longitudinal = ox * math.cos(heading) + oy * math.sin(heading)
            lateral = -ox * math.sin(heading) + oy * math.cos(heading)
            clearance = math.hypot(max(abs(longitudinal) - car_length / 2, 0),
                                   max(abs(lateral) - car_width / 2, 0)) - obstacle["radius"]
            minimum_clearance = min(minimum_clearance, clearance)
            if clearance <= motion_margin:
                collision_detected = True

    lead_x_positions.append(lead_car_x)
    lead_y_positions.append(lead_car_y)

    following_x_positions.append(following_car_x)
    following_y_positions.append(following_car_y)


    if step % 10 == 0:

        print(round(step * t, 1), 
              round(gap, 2), 
              round(safe_gap, 2), 
              round(target_speed, 2),
              round(following_car_v),
              round(acceleration, 3))

    if collision_detected:
        print("COLLISION DETECTED: scenario stopped; do not interpret as successful avoidance.")
        break

print("Obstacle collision:", collision_detected)
print("Minimum obstacle surface clearance (m):", minimum_clearance)
print("Final speed (m/s):", following_car_v)
# Scope: straight-road obstacle prototype. Curved-road path prediction and group
# morphology/lane-change/roundabout integration remain pending. Objects are static;
# perfect position knowledge and dry-road force limits are assumed. A late object
# can be unavoidable. Original teammate source files are not edited by this copy.
