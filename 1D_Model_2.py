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
leader_x = 3 #position

following_v = 0 # speed
following_x = 0 # position

d_0 = 3 #minimum gap of 3metres
T_h = 2 # time taken for the following car at the current speed to reach the lead car
v_max = 70 * 0.44704    #speed limit in m/s

pid_freq = 10   #number of pid updates per second
time_step = 1 / pid_freq    #the time duration in seconds between each iteration
summed_distance_error = 0   #used in the integral part of the pid
distance_error = 0  #initialise
summed_velcoity_error = 0
velocity_error = 0

#constants for desired velocity pid controller
J_p = 1
J_i = 0.1
J_d = 0.2

#constants for acceleration pid controller
K_p = 1
K_i = 0.1
K_d = 0.2

#list of plotting values
distance_list = np.zeros(leader_updates_num*pid_freq)
following_v_list = np.zeros(leader_updates_num*pid_freq)
leader_v_new_list = np.zeros(leader_updates_num*pid_freq)


#updates values more times than there is data values for the leader (10 times per second for this example)
for i in range(pid_freq * leader_updates_num):
    time = time_step*i  #time elapsed
    leader_v = np.interp(time, df["time_s"], leader_v_list) #creates values for the leader velocity in between the second intervals we have (only done linearly so not very accurate)




    #
    #Desired Velocity PID
    #

    d_desired = d_0 + (T_h * following_v)   #equation for a desired distance

    d = leader_x - following_x #equation for actual distance
    roc_distance_error = ((d-d_desired) - distance_error) / time_step   #used in the derivative part of pid - is the rate of change of error between last time step and now
    distance_error = d - d_desired  #updates distance error

    
    velocity_pid = (J_p*distance_error) + (J_i * summed_distance_error) + (J_d * roc_distance_error)    #test velocity pid, used for seeing what it wants us to do next

    upper_saturation = velocity_pid >= v_max    #pid wants us to go faster than speed limit 
    lower_saturation = velocity_pid < 0         #pid wants us to go backwards

     
    #sum error in any instance apart from the state where we're (desired to go faster than speed limit and the distance is greater than desired)
    #   -in other words when the pid wants us to go faster than we're allowed and the gap is bigger than we'd want, there is nothing we can do, so no point adding and adding to the massive error it creates
    #Also a negative situation that stops error being accumulated when we're asked to negatively move, cannot happen, but we're already at a smaller than desired seperation gap so again no point summing up errors.
    #WHY - for example when we hit a speed limit and the leader drives far off into the distance for a while, if we let an integral error sum the whole of this time it will become so huge that if the
    #car in front rapidly breaks our car wont react because this integral term will dominate in telling the car that it must make up for a forward error still
    if not(upper_saturation and distance_error > 0) and not(lower_saturation and distance_error < 0):
        summed_distance_error += distance_error*time_step #integral sumation over time of distance error

    velocity_pid = (J_p*distance_error) + (J_i * summed_distance_error) + (J_d * roc_distance_error)    #main velocity_pid


    v_desired = np.clip(velocity_pid, 0, v_max)    #desired velocity, is constrained because we can with a desired velocity (we would never aim to go above speed limit) 

    #
    #
    #



    #
    #Acceleration PID
    #

    roc_velocity_error = ((v_desired - following_v) - velocity_error) / time_step   #rate of change of velocity error
    velocity_error = v_desired - following_v    #updates velocty error


    acceleration_desired = (K_p*velocity_error) + (K_i*summed_velcoity_error) + (K_d*roc_velocity_error)    #acceleeration test pid

    upper_saturation = acceleration_desired >= 3.5    #we are desired to accelerate faster than possible
    lower_saturation = acceleration_desired < -8      #we are desired to decelearte harder than possible

    #same premise really, only sums if not the case of: we are desired to accelerate more than possible and velocity error is positive meaning we will accumulate integral error
    #                                                    - we are desired to decelerate more than possible and we're going faster than desired
    if not(upper_saturation and velocity_error > 0) and not(lower_saturation and velocity_error < 0):
        summed_velcoity_error += velocity_error*time_step #integral sumation

    acceleration_desired = np.clip(((K_p*velocity_error) + (K_i*summed_velcoity_error) + (K_d*roc_velocity_error)), -8, 3.5)    #main acceleeration pid - constrained between realistic values

    #
    #
    #



    
    #updates positions and velocities
    following_v = np.clip((following_v + acceleration_desired*time_step), 0, v_max)
    leader_x = leader_x + leader_v*time_step
    following_x = following_x + following_v*time_step

    distance_list[i] = d
    following_v_list[i] = following_v
    leader_v_new_list[i] = leader_v

    if(d<0):
        print("crash", i/pid_freq)
        

time_points = np.linspace(0, leader_updates_num, num=leader_updates_num*pid_freq)
fig, axs = plt.subplots(2)
axs[0].plot(time_points, distance_list)
axs[0].axhline(y=0, color="r")
axs[0].set_title("Seperation Distance")
axs[1].plot(time_points, following_v_list, label="Following Velocity")
axs[1].plot(time_points, leader_v_new_list, label="Leader Velocity")
axs[1].legend()

plt.show()
    





