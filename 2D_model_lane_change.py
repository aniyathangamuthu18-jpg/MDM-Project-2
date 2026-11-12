import math
import matplotlib.pyplot as plt

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

    gap = lead_car_x - following_car_x # calculate current gap

    safe_gap = min_gap + (time_headway * following_car_v) # calculate desired safe gap

    gap_error = gap - safe_gap

    target_speed = lead_car_v + (K_gap * gap_error)

    error = target_speed - following_car_v # speed error
    P = K_p * error # proportional term

    integral_error += (error * t) # integral term
    I = K_i * integral_error

    D = K_d * (error - previous_error) / t # derivative term

    acceleration = P + I + D # total PID acceleration

    previous_error = error # save error for next time step

    old_following_car_v = following_car_v
    following_car_v = old_following_car_v + (acceleration * t) # updated speed

    direction_x = dx/gap
    direction_y = dy/gap # calcuates the direction towards the lead car for every 1m travelled

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


###

current_lane = 0
right_lane = 1
left_lane = 2

lane_width = 3.5
lane_change_duration = 4
lane_positions ={0: 0, 1: 3.5, 2: -3.5}

desired_speed = 25

following_car_v = 12
following_car_x = 30
following_car_y = 0

lead_car_v = 15
lead_car_x = 50

min_gap = 5
time_headway = 1.5

t = 0.1

vehicles = [
    {"x": 50, "v": 15, "lane": 0},  #vehicle infront, current lane
    {"x": 90, "v": 20, "lane": 1},  #vehicle ahead, right lane
    {"x": 20, "v": 18, "lane": 1},  #vehicle behind, right lane
    {"x": 40, "v": 19, "lane": 2},  #vehicle ahead, left lane
    {"x": 20, "v": 16, "lane": 2}   #vehicle behind, left lane
]


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


lane_state = "FOLLOWING"

original_lane = current_lane
target_lane = None

lane_change_elapsed = 0




following_x_history = []
following_y_history = []

lead_x_history = []

vehicles_history = []
time_history = []



for step in range(500):

    for vehicle in vehicles:

        vehicle["x"] += vehicle["v"] * t

    if lane_state == "FOLLOWING":

        if lead_car_v < desired_speed:

            right_safe = lane_change_decision(
                following_car_x,
                following_car_v,
                right_lane,
                vehicles
            )

            left_safe = lane_change_decision(
            following_car_x,
            following_car_v,
            left_lane,
            vehicles
        )

        if right_safe and left_safe:

            right_front, right_rear, right_front_gap, right_rear_gap = (
                find_nearest_vehicles(
                    following_car_x, right_lane, vehicles
                )
            )

            left_front, left_rear, left_front_gap, left_rear_gap = (
                find_nearest_vehicles(
                    following_car_x, left_lane, vehicles
                )
            )

            right_clearance = min(right_front_gap, right_rear_gap)
            left_clearance = min(left_front_gap, left_rear_gap)

            if right_clearance > left_clearance:

                target_lane = right_lane

            else:
                target_lane = left_lane

        elif right_safe:

            target_lane = right_lane

        elif left_safe:

            target_lane = left_lane

        else:

            target_lane = None

        if target_lane is not None:

            lane_state = "CHANGING_LANE"

            original_lane = current_lane

            start_y = following_car_y

            target_y = lane_positions[target_lane]

            lane_change_elapsed = 0


    if lane_state == "CHANGING_LANE":

        lane_change_elapsed += t

        following_car_y = calculate_lane_position(
            start_y,
            target_y,
            lane_change_elapsed,
            lane_change_duration
        )

        if lane_change_elapsed >= lane_change_duration:

            current_lane = target_lane 

            lane_state = "OVERTAKING"

    if lane_state == "OVERTAKING":

        if following_car_x > lead_car_x + min_gap:

            return_safe = lane_change_decision(
                following_car_x,
                following_car_v,
                original_lane,
                vehicles
            )

            if return_safe:

                target_lane = original_lane
                lane_state = "CHANGING_BACK"

                start_y = following_car_y

                target_y = lane_positions[target_lane]

                lane_change_elapsed = 0

    if lane_state == "CHANGING_BACK":

        lane_change_elapsed +=  t

        following_car_y = calculate_lane_position(
            start_y,
            target_y,
            lane_change_elapsed,
            lane_change_duration
        )

        if lane_change_elapsed >= lane_change_duration:

            current_lane = original_lane

            lane_state = "FOLLOWING"

            target_lane = None

    following_car_x += following_car_v * t

    lead_car_x += lead_car_v * t


    if step % 10 == 0:

        print(
            round(step * t, 1),
            round(following_car_x, 2),
            round(following_car_y, 2),
            current_lane,
            lane_state
        )

    
    following_x_history.append(following_car_x)
    following_y_history.append(following_car_y)

    lead_x_history.append(lead_car_x)

    vehicles_history.append(
        [(vehicle["x"], lane_positions[vehicle["lane"]])
        for vehicle in vehicles]
    )

    time_history.append(step * t)

plt.figure(figsize=(10, 5))

plt.plot(
    time_history,
    following_y_history,
    label="Following car lateral position",
    linewidth=2
)

plt.xlabel("Time (s)")
plt.ylabel("Lateral position (m)")

plt.title("Vehicle Lateral Position During Lane Change")

plt.yticks([-3.5, 0, 3.5],
           ["Left lane", "Current lane", "Right lane"])

plt.grid(True)

plt.legend()

plt.tight_layout()

plt.show()


           

    

