import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

#helped me find my data file maybe ignore for you :)
folder = Path(__file__).resolve().parent

#read csv into a data frame and creates a list of the speeds
df = pd.read_csv(folder / "Velocity_Data.csv")
leader_v_list = df["speed_mph"].copy().to_numpy()
leader_v_list = leader_v_list * 0.44704    #converts from mph to m/s
leader_updates_num = len(leader_v_list) #simply gives the amount of velocity data points there are

leader_v = 0  #speed
leader_x = 5 #position

following_v = 0 # speed
following_x = 0 # position

d_0 =5 #minimum gap between cars
T_h = 2.5 # time headway between cars
v_max = 70 * 0.44704    #speed limit in m/s

pid_freq = 10   #number of pid updates per second
time_step = 1 / pid_freq    #the time duration in seconds between each iteration

total_iterations = pid_freq * leader_updates_num    

#initialise variables for pid
distance_error = (leader_x - following_x) - (d_0 + T_h*following_v)
summed_velcoity_error = 0
velocity_error = 0


#constants for acceleration pid controller
K_p = 0.4
K_i = 0.0
K_d = 0.0

# distance control gain
k_distance = 0.5

#list of plotting values
distance_list = np.zeros(total_iterations)
desired_distance_list = np.zeros(total_iterations)
following_v_list = np.zeros(total_iterations)
leader_v_new_list = np.zeros(total_iterations)
#performance variables
crash_count = 0
distance_error_list = np.zeros(total_iterations)
TTC_list = np.full(total_iterations, np.nan)
accleration_list = np.zeros(total_iterations)

# Resistive force parameters
m = 1500
rho = 1.225
C_d = 0.3
A = 2.2
C_rr = 0.015
g = 9.81



#updates values more times than there is data values for the leader (10 times per second for this example)
for i in range(total_iterations):
    time = time_step*i  #time elapsed
    leader_v = np.interp(time, df["time_s"], leader_v_list) #creates values for the leader velocity in between the second intervals we have (only done linearly so not very accurate)

    d = leader_x - following_x #distance between two cars
    d_desired = d_0 + (T_h * following_v)   #desired distance    
    
    distance_error = d - d_desired  
    distance_error_list[i] = distance_error

    #desired velocity must be constrained for obvious reasons
    v_desired = np.clip(leader_v + k_distance*distance_error, 0, v_max)


    #
    #Acceleration PID
    #

    #avoids a startup jump because error will have gone from 0 to whatever
    if(i==0):
        roc_velocity_error = 0
    else:
        roc_velocity_error = ((v_desired - following_v) - velocity_error) / time_step   #rate of change of velocity error

    velocity_error = v_desired - following_v    #updates velocty error

    acceleration_desired = (K_p*velocity_error) + (K_i*summed_velcoity_error) + (K_d*roc_velocity_error)    #acceleeration test pid

    upper_saturation = acceleration_desired >= 2    #we are desired to accelerate faster than possible
    lower_saturation = acceleration_desired < -8      #we are desired to decelearte harder than possible

    #same premise really, only sums if not the case of: we are desired to accelerate more than possible and velocity error is positive meaning we will accumulate integral error
    #                                                    - we are desired to decelerate more than possible and we're going faster than desired
    #                                                    - basically dont sum if we are asked to accelerate more or less than our constraints, and 
    if not(upper_saturation and velocity_error > 0) and not(lower_saturation and velocity_error < 0):
        summed_velcoity_error += velocity_error*time_step #integral sumation

    acceleration = np.clip(((K_p*velocity_error) + (K_i*summed_velcoity_error) + (K_d*roc_velocity_error)), -8, 2)    #main acceleeration pid - constrained between realistic values

    #
    #
    #

    # Resistive decelerations
    a_drag = (0.5 * rho * C_d * A * following_v**2) / m
    a_rr = C_rr * g

    # Simplified resistance threshold at rest
    if following_v <= 0 and acceleration <= a_rr:
        a_actual = 0.0
    else:
        a_actual = acceleration - a_drag - a_rr

    #plotting lists
    distance_list[i] = d
    desired_distance_list[i]= d_desired
    following_v_list[i] = following_v
    leader_v_new_list[i] = leader_v
    accleration_list[i] = a_actual

    if(following_v > leader_v and d>0):
        TTC_list[i] = d / (following_v-leader_v)
    else:
        TTC_list[i] = np.nan
    
    #updates positions and velocities
    old_v = following_v
    a = a_actual

    #this looks confusing but if negatively acclerating then between two time steps following_v could become negative therefore 
    #this moves the car only for the time required to stop and then sets velocity =0
    if a < 0 and old_v + a*time_step < 0:
        stopping_time = -old_v/a

        following_x += (old_v*stopping_time + 0.5*a*stopping_time**2)

        following_v = 0
    else:
        following_x += (old_v*time_step + 0.5*a*time_step**2)

        following_v = old_v + a*time_step


    leader_v_next = np.interp(time + time_step, df["time_s"], leader_v_list)
    leader_x += 0.5*(leader_v + leader_v_next)*time_step

    

    new_d = leader_x - following_x

    if(d>0 and new_d<=0):
        crash_count += 1

  
        


time_points = np.arange(len(distance_list)) * time_step

# Three vertically stacked plots with a shared time axis
fig, axs = plt.subplots(
    3, 1,
    figsize=(11, 9),
    sharex=True,
    constrained_layout=True
)

# Consistent formatting for all three plots
for ax in axs:
    ax.grid(True, linestyle="--", alpha=0.3)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.tick_params(labelsize=14)

# 1. Separation distance
axs[0].plot(time_points, distance_list, label="Actual separation", color="tab:blue", linewidth=2.2)

axs[0].plot(time_points, desired_distance_list, label="Desired separation", color="tab:orange", linestyle="--", linewidth=2.2)

axs[0].axhline(0, color="red", linewidth=1.9, alpha=0.6)
axs[0].set_title("Vehicle Separation Distance", fontweight="bold")
axs[0].set_ylabel("Distance (m)")
axs[0].legend(loc="upper right", frameon=False)

# 2. Vehicle velocities
axs[1].plot(time_points, following_v_list, label="Follower velocity", color="tab:blue", linewidth=2.2)

axs[1].plot(time_points, leader_v_new_list, label="Leader velocity", color="tab:orange", linestyle="--", linewidth=2.2)

axs[1].set_title("Leader and Follower Velocities", fontweight="bold")
axs[1].set_ylabel("Velocity (m/s)")
axs[1].legend(loc="upper right", frameon=False)

# 3. Acceleration
axs[2].plot(time_points, accleration_list, color="tab:green", linewidth=2.2)

axs[2].axhline(0, color="black", linestyle="--", linewidth=1.5, alpha=0.5)

axs[2].set_title("Follower Commanded Acceleration", fontweight="bold")
axs[2].set_ylabel("Acceleration (m/s²)")
axs[2].set_xlabel("Time (s)")

# Overall heading
fig.suptitle("Adaptive Cruise Control Simulation Results", fontsize=17, fontweight="bold")



plt.show()
    

min_seperation = min(distance_list)
below_desired_seperation = (len([i for i in distance_error_list if i<0]) / len(distance_error_list)) * 100
distance_rmse = (np.sqrt(np.mean(distance_error_list**2)))
unsafe_error = np.minimum(distance_error_list, 0)
negative_error_rmse = np.sqrt(np.mean(unsafe_error**2))#
jerk_list = np.diff(accleration_list) / time_step
rms_acceleration = np.sqrt(np.mean(accleration_list**2))

print(f"Distance RMSE: {distance_rmse:.2f}")
print(f"Unsafe Distance RMSE: {negative_error_rmse:.2f}")
print(f"Minimum Seperation: {min_seperation}m")
print(f"Total Crash Count: {crash_count}")
print(f"Time below Desired Seperation: {below_desired_seperation:.2f}%")
print(f"Maximum Jerk: {np.max(np.abs(jerk_list)):.2f}")
print(f"RMS Acceleration: {rms_acceleration:.2f} m/s²")