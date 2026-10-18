import math

# speed (v) measured in m/s
# position (x) measured in m
# time measured in s
# e(t) = lead_car_v(t) - following_car_v(t)
# p(t) = K_p * e(t)
# i(t) = K_i * integral e(t)
# d(t) = K_d  * derivative e(t)
# following_car_a = p(t) + i(t) + d(t)

lead_car_v = 20 # speed
lead_car_x = 50 # x position
lead_car_y = 50 # y position
following_car_v = 10 # speed
following_car_x = 0 #  x position
following_car_y = 0 # y position
min_gap = 5 # minimum gap when both cars are at a standstill - whatever speed they are travelling at, will always be at least a 5m gap for safety
time_headway = 1.5 # time taken for the following car at the current speed to reach the lead car
K_gap = 0.1 # controls how strongly the following car responds to the distance between two cars

t = 0.1 # time
K_p = 0.5 # proportional controller - responds to current speed error
K_i = 0.05 # integral controller - accumulates speed error over time
K_d = 0.5 # derivative controller - responds to how quickly error changes
integral_error = 0 # initially set to zero 
previous_error = lead_car_v - following_car_v

for step in range(100):

    dx = lead_car_x - following_car_x
    dy = lead_car_y - following_car_y

    gap = math.sqrt(dx**2 + dy**2) # calculate current gap

    safe_gap = min_gap + (time_headway * following_car_v) # calculate desired safe gap

    gap_error = gap - safe_gap

    target_speed = lead_car_v + (K_gap * gap_error)

    error = target_speed - following_car_v # speed error
    P = K_p * error # proportional term

    integral_error = integral_error + (error * t) # integral term
    I = K_i * integral_error

    D = K_d * (error - previous_error) / t # derivative term

    acceleration = P + I + D # total PID acceleration

    previous_error = error # save error for next time step

    old_following_car_v = following_car_v
    following_car_v = old_following_car_v + (acceleration * t) # updated speed

    direction_x = dx/gap
    direction_y = dy/gap # calcuates the direction towards the lead car

    lead_car_x = lead_car_x + (lead_car_v * t) # update lead car position

    following_car_x = following_car_x + (old_following_car_v * direction_x * t) # update following car position
    following_car_y = following_car_y + (old_following_car_v * direction_y * t)

    if step % 10 == 0:

        print(round(step * t, 1), 
              round(gap, 2), 
              round(safe_gap, 2), 
              round(target_speed, 2),
              round(following_car_v),
              round(acceleration, 3))