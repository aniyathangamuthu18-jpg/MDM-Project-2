import math
import random

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
lead_car_y = 50 # y position
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


# obstacles are spawned randomly during the simulation
obstacles = []

# Approximate obstacle sizes (metres)
obstacle_types = [
    ("small_debris", 0.4),
    ("large_debris", 0.8),
    ("stopped_vehicle", 1.2),
]

obstacle_spawn_probability = 0.02  # chance of spawning on each time step
obstacle_detection_range = 50.0
lane_half_width = 1.8


# Actuator parameters, adding small delay 
tau_actuator = 0.3
actual_acceleration = 0.0


for step in range(100):

    dx = lead_car_x - following_car_x
    dy = lead_car_y - following_car_y

    gap = math.sqrt(dx**2 + dy**2) # calculate current gap

    safe_gap = min_gap + (time_headway * following_car_v) # calculate desired safe gap

    gap_error = gap - safe_gap

    # changes made to the code to include obstacle detection and avoidance:
    # find the closest obstacle ahead within the car's path
    closest_obstacle_target_speed = None

    # Use the available braking force to estimate a conservative stopping speed
    obstacle_braking_acceleration = min(
        max_deceleration,
        max_braking_force / mass
    )
    reaction_time = time_headway + tau_actuator + t # include actuator delay in obstacle stopping logic


    # randomly choose an obstacle and place it ahead of the car
    if random.random() < obstacle_spawn_probability:
        obstacle_type, obstacle_radius = random.choice(obstacle_types)
        spawn_distance = random.uniform(15.0, 40.0)
        lateral_offset = random.choice([-4.0, 0.0, 4.0])

        obstacles.append({
            "type": obstacle_type,
            "x": (
                following_car_x
                + spawn_distance * math.cos(following_car_heading)
                - lateral_offset * math.sin(following_car_heading)
            ),
            "y": (
                following_car_y
                + spawn_distance * math.sin(following_car_heading)
                + lateral_offset * math.cos(following_car_heading)
            ),
            "radius": obstacle_radius
        })

    for obstacle in obstacles:
        obstacle_dx = obstacle["x"] - following_car_x
        obstacle_dy = obstacle["y"] - following_car_y

        # Resolve obstacle position into forward and sideways distances
        forward_distance = (
            obstacle_dx * math.cos(following_car_heading)
            + obstacle_dy * math.sin(following_car_heading)
        )
        lateral_distance = (
            -obstacle_dx * math.sin(following_car_heading)
            + obstacle_dy * math.cos(following_car_heading)
        )

        # Relevant if it is nearby, ahead, and within the lane/path
        if (
            0 < forward_distance <= obstacle_detection_range
            and abs(lateral_distance) <= lane_half_width + obstacle["radius"]
        ):
            stopping_distance_available = max(
                0.0,
                forward_distance - obstacle["radius"] - min_gap
            )

            safe_obstacle_speed = obstacle_braking_acceleration * (
                math.sqrt(
                    reaction_time**2
                    + 2 * stopping_distance_available / obstacle_braking_acceleration
                )
                - reaction_time
            )

            if (
                closest_obstacle_target_speed is None
                or safe_obstacle_speed < closest_obstacle_target_speed
            ):
                closest_obstacle_target_speed = safe_obstacle_speed
    

    # Cap the speed if an obstacle is ahead
    target_speed = lead_car_v + (K_gap * gap_error)

    if closest_obstacle_target_speed is not None:
        target_speed = min(target_speed, closest_obstacle_target_speed)

    target_speed = max(0.0, target_speed)

    # end of changes for obstacle detection and avoidance

    error = target_speed - following_car_v # speed error
    P = K_p * error # proportional term

    integral_error = integral_error + (error * t) # integral term
    I = K_i * integral_error

    D = K_d * (error - previous_error) / t # derivative term

    acceleration = P + I + D # total PID acceleration

    previous_error = error # save error for next time step

    old_following_car_v = following_car_v

    desired_acceleration = max(-max_deceleration, min(acceleration, max_acceleration)) # sets limits on acceleration to what an average car can do
    actual_acceleration += (t / tau_actuator) * (desired_acceleration - actual_acceleration) # adds a small delay to the acceleration to simulate actuator delay

    engine_force = mass * max(actual_acceleration, 0) # calculates engine force with F = ma, max function ensures engine force has positive acceleration
    engine_force = min(engine_force, max_engine_force) # sets limit of max engine force average car can produce

    braking_force = mass * max(-actual_acceleration, 0)  #calculate braking force with F = ma, max function ensures braking force is calculated when acceleration is negative
    braking_force = min(braking_force, max_braking_force) #sets limit of braking force

    drag_force = 0.5 * air_density * drag_coeff * front_area * following_car_v**2 # calcuate air drag force
    
    friction_force = friction_factor * mass * 9.81 # calculates friction force between tires and ground

    net_force = (engine_force - braking_force - drag_force - friction_force) # combines forces

    physical_acceleration = net_force / mass #calculates a new acceleration based on physics of car
    following_car_v = max(0.0,old_following_car_v + physical_acceleration * t) # updates speed of following car based on physics, plus no reversing

    desired_heading = math.atan2(dy, dx) # calcuates angle between cars
    heading_error = desired_heading - following_car_heading 

    steering_angle = k_steering * heading_error # uses controller to find steering angle

    angular_velocity = (following_car_v / wheelbase* math.tan(steering_angle)) #calculates angular velocity

    lead_car_x = lead_car_x + (lead_car_v * t) # update lead car position currently just have lead car driving straight

    old_following_heading = following_car_heading
    following_car_heading = following_car_heading + angular_velocity * t #updates following car heading after timestep

    average_heading = (old_following_heading + following_car_heading) / 2
    distance_travelled = 0.5 * (old_following_car_v + following_car_v) * t

    following_car_x += (distance_travelled * math.cos(average_heading))
    following_car_y += (distance_travelled * math.sin(average_heading)) #updates position after timestep

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
