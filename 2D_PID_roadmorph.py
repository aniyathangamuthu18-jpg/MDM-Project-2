import math
import matplotlib.pyplot as plt

#constants used to calculate forces acting on the car
mass = 1500 

max_engine_force = 4000
max_braking_force = 8000

drag_coeff = 0.3
front_area = 2.2
air_density = 1.225
friction_factor = 0.015

max_acceleration = max_engine_force/mass
max_deceleration = max_braking_force/mass

max_lateral_acceleration = 2.5
max_road_speed = 25

#car parameters
following_car_distance = 0
lead_car_distance = 50

lead_car_v = 10
following_car_v = 10
previous_target_speed = following_car_v

min_gap = 5
time_headway = 1.5
K_gap = 0.1

#PID parameters
t = 0.1

K_p = 0.5
K_i = 0.05
K_d = 0.5

look_ahead_distance = 3 #distance car looks ahead for bend in meters

def get_road_curvature(distance): #simulates a straight road with with a bend after 50 meters then straight

    if distance < 50:
        return 0          # Straight before bend
    elif distance < 80:
        return 1 / 30     # Left bend, radius 30m
    else:
        return 0          # Straight after bend

def calculate_road_speed_limit(curvature): #limits speed of car to road limit if road is straight, speed is limited when car is going around bend

    if curvature == 0:
        return max_road_speed
    return min(
        math.sqrt(max_lateral_acceleration / abs(curvature)),max_road_speed)

def get_road_position(distance): #calculates the position of the car and the angle the car is facing at its point on the road
    if distance < 50:

        x = distance
        y = 0
        heading = 0 # calculates position during straight before the bend
    elif distance < 80:

        angle = (distance - 50) / 30

        x = 50 + 30 * math.sin(angle)
        y = 30 * (1 - math.cos(angle))

        heading = angle # calculates position during left bend (50-80m)
    else:

        x_start = 50 + 30 * math.sin(1)
        y_start = 30 * (1 - math.cos(1))

        x = x_start + (distance - 80) * math.cos(1)
        y = y_start + (distance - 80) * math.sin(1)

        heading = 1 # calculates position during straight after bend

    return x, y, heading

def calculate_braking_speed(distance): # This is used to calculate the speed the car should be to driving around the bend in an energy efficent way

    bend_speed = calculate_road_speed_limit(1 / 30) #max speed car can travel around the bend

    if distance < 50:

        distance_to_bend = 50 - distance
        safe_speed = math.sqrt(bend_speed**2+ 2 * max_deceleration * distance_to_bend)

        return min(safe_speed, max_road_speed) #speed that energy efficient car should follow leading up to the bend

    elif distance < 80: 
        return bend_speed #this is the speed the car can go around the bend

    else:
        return max_road_speed #after the bend on the straight road car can return to max speed

def simulate(energy_efficient): #for loop used in previous models placed inside of function so it can be simulated with and without energy efficient cruise control

    lead_distance = 50 #initial variable before car have started driving
    following_distance = 0

    lead_v = 20
    following_v = 10

    integral_error = 0

    previous_error = lead_v - following_v

    engine_energy = 0
    braking_energy = 0

    lead_x_positions = [] #results stores
    lead_y_positions = []

    following_x_positions = []
    following_y_positions = []

    following_speeds = []


    for step in range(75):

        #calculate lead car speed and position after each timestep
        lead_safe_speed = calculate_braking_speed(lead_distance)

        if lead_v > lead_safe_speed:
            lead_v = max(lead_v - max_deceleration * t,lead_safe_speed)

        else:
            lead_v = min(lead_v + max_acceleration * t,lead_safe_speed)

        lead_distance += lead_v * t

        lead_x, lead_y, lead_heading = get_road_position(lead_distance)
        

        gap = lead_distance - following_distance #calculates current gap
        safe_gap = min_gap + time_headway * following_v #calculates desired safe gap
        gap_error = gap - safe_gap 

        target_speed = lead_v + K_gap * gap_error # Calculate desired following speed

        if energy_efficient: #run to calculate values for car that modulates speed for bend in efficiently

            safe_speed = calculate_braking_speed(following_distance) #uses previous function
            target_speed = min(target_speed, safe_speed) #choose efficient safe speed when it is lower than the target speed

        else: #uses original PID controller to slow down to safe speed just before the bend
            future_distance = (following_distance + look_ahead_distance)

            curvature = get_road_curvature(future_distance)

            road_speed_limit = calculate_road_speed_limit(curvature)

            target_speed = min(target_speed,road_speed_limit)

        error = target_speed - following_v

        P = K_p * error # PID controller same in all models
        integral_error += error * t
        I = K_i * integral_error
        D = K_d * (error - previous_error) / t

        acceleration = P + I + D
        desired_acceleration = max(-max_deceleration,min(acceleration, max_acceleration)) #limits acceleration to what a car can do
        previous_error = error

        old_following_v = following_v #calculates the vehicle forces to make model take the physics into account

        engine_force = mass * max(desired_acceleration, 0) # calculates engine force with F = ma, max function ensures engine force has positive acceleration
        engine_force = min(engine_force,max_engine_force) # sets limit of max engine force average car can produce

        braking_force = mass * max(-desired_acceleration, 0) #calculates braking forces
        braking_force = min(braking_force,max_braking_force) #sets limits of max braking force

        drag_force = (0.5 * air_density * drag_coeff *front_area * following_v**2) # calculates other forces acting on the car
        friction_force = (friction_factor * mass * 9.81)
        net_force = (engine_force- braking_force- drag_force- friction_force)

        physical_acceleration = net_force / mass

        following_v = (old_following_v + physical_acceleration * t)

        distance_travelled = ((old_following_v + following_v) / 2) * t #used to calculate energy used up and released by car

        engine_energy += (engine_force * distance_travelled) # Engine energy = force x distance

        braking_energy += (braking_force * distance_travelled) # Braking energy = braking force x distance

        following_distance += distance_travelled #update following car position

        following_x, following_y, following_heading = (get_road_position(following_distance))

        # Store positions

        lead_x_positions.append(lead_x)
        lead_y_positions.append(lead_y)

        following_x_positions.append(following_x)
        following_y_positions.append(following_y)

        following_speeds.append(following_v)
    
    return {
        "lead_x": lead_x_positions,
        "lead_y": lead_y_positions,

        "following_x": following_x_positions,
        "following_y": following_y_positions,

        "speeds": following_speeds,

        "engine_energy": engine_energy,
        "braking_energy": braking_energy,} #simulation results


original = simulate(False)
efficient = simulate(True)

print("Original PID controller")

print("Engine energy:",round(original["engine_energy"] / 1000000, 3), "MJ")
print("Braking energy:",round(original["braking_energy"] / 1000000, 3), "MJ")


print("Energy-efficient controller")

print("Engine energy:",round(efficient["engine_energy"] / 1000000, 3), "MJ")
print("Braking energy:",round(efficient["braking_energy"] / 1000000, 3), "MJ")

plt.figure(figsize=(10, 6))

plt.plot(
    original["lead_x"],
    original["lead_y"],
    label="Lead car"
)

plt.plot(
    original["following_x"],
    original["following_y"],
    label="Original PID"
)

plt.plot(
    efficient["following_x"],
    efficient["following_y"],
    "--",
    label="Energy-efficient PID"
)

plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")

plt.title("Vehicle trajectories")

plt.legend()
plt.axis("equal")
plt.grid()

plt.show()