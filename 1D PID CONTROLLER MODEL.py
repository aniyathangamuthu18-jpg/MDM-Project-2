# speed (v) measured in m/s
# position (x) measured in m
# time measured in s
# e(t) = lead_car_v(t) - following_car_v(t)
# p(t) = K_p * e(t)
# i(t) = K_i * integral e(t)
# d(t) = K_d  * derivative e(t)
# following_car_a = p(t) + i(t) + d(t)

# Adding random variatons in speed of lead car and slow/stop/restarts
import random

speed_limit = 20.0
stop_probability = 0.001  # 0.1% chance per driving timestep

lead_state = "driving"   # driving, stopping or waiting each are a state, we are going to start with the driving state
restart_time = None # this is random, initalise here

# Initialisaiton
lead_car_v = 20 # speed
lead_car_x = 50 # position
following_car_v = 10 # speed
following_car_x = 0 # position
min_gap = 5 # minimum gap when both cars are at a standstill - whatever speed they are travelling at, will always be at least a 5m gap for safety
time_headway = 1.5 # time taken for the following car at the current speed to reach the lead car
K_gap = 0.1 # controls how strongly the following car responds to the distance between two cars


t = 0.1 # time step
K_p = 0.5 # proportional controller - responds to current speed error
K_i = 0.05 # integral controller - accumulates speed error over time
K_d = 0.5 # derivative controller - responds to how quickly error changes
integral_error = 0 # initially set to zero 
previous_error = lead_car_v - following_car_v


# Adding drag to the 1D Model:
air_density = 1.225   # Standard atmosphere air density at sea level
drag_coefficient = 0.3 # this is a random number, describing how aerodynamic the car’s shape is. Lower means less drag
frontal_area = 2.2    # this is a random number, making an assumption
car_mass = 1500       # this is a random number, making an assumption


for step in range(1000):

    # Add the random variatons in speed of lead car and slow/stop/restarts:

    current_time = step * t

    # Randomly begin stopping while driving
    if lead_state == "driving":
        if random.random() < stop_probability: # random.random() prodcues a number between 0 and 1
            lead_state = "stopping"

    # Restart after the waiting period
    elif lead_state == "waiting":
        if current_time >= restart_time:
            lead_state = "driving"

    # Choose target speed
    if lead_state == "driving":
        lead_target_speed = speed_limit + random.uniform(-0.3, 0.0) # while driving will have some variations
    else:
        lead_target_speed = 0.0

    # Approach target gradually, with acceleration/braking limits
    lead_acceleration = 0.5 * (lead_target_speed - lead_car_v)
    lead_acceleration = max(-1.5, min(1.5, lead_acceleration)) # maxiumu and minimum acceleration and deceleration of 1.5 and 1.5


    # PID Controller:

    gap = lead_car_x - following_car_x # calculate current gap

    safe_gap = min_gap + (time_headway * following_car_v) # calculate desired safe gap

    gap_error = gap - safe_gap

    target_speed = lead_car_v + (K_gap * gap_error)
    target_speed = max(0.0, min(speed_limit, target_speed)) # to keep the following car at the speed limit as well

    error = target_speed - following_car_v # speed error
    P = K_p * error # proportional term

    integral_error = integral_error + (error * t) # integral term
    I = K_i * integral_error

    D = K_d * (error - previous_error) / t # derivative term


    controller_acceleration = P + I + D

    # Add the change of the acceleration due to drag:

    drag_force = (
        0.5 * air_density * drag_coefficient
        * frontal_area * following_car_v**2) # this is the drag formula in the word document

    drag_acceleration = drag_force / car_mass # this is f=ma, also in the word document

    acceleration = controller_acceleration - drag_acceleration # explained in the word document if needed but this is f=ma as well. 

    # Updating for the next step

    previous_error = error # save error for next time step

    old_following_car_v = following_car_v
    #following_car_v = old_following_car_v + (acceleration * t) # updated speed
    following_car_v = max(0.0, old_following_car_v + acceleration * t) # if the following car v below 0, will choose 0 so no reversing
    lead_car_x = lead_car_x + (lead_car_v * t) # update lead car position
    following_car_x = following_car_x + (old_following_car_v * t) # update following car position

    # Update lead-car speed for the next timestep
    lead_car_v = max(0.0, lead_car_v + lead_acceleration * t) # if the lead car v below 0, will choose 0 so no reversing

    # Once fully stopped, choose the waiting time once
    if lead_state == "stopping" and lead_car_v < 0.05:
        lead_car_v = 0.0
        lead_state = "waiting"

        waiting_time = random.uniform(2.0, 10.0)
        restart_time = (step + 1) * t + waiting_time





    if step % 10 == 0:

        print(round(step * t, 1), 
              round(gap, 2), 
              round(safe_gap, 2), 
              round(target_speed, 2),
              round(following_car_v),
              round(acceleration, 3))
