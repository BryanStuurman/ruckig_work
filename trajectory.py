#!/usr/bin/env python3
from ruckig import InputParameter, OutputParameter, Result, Ruckig, Synchronization, Interface, DurationDiscretization
from math import sin, cos

ts=1/50 # 50Hz robot

# Basic parameters
v_limits=[.25, .07]
a_limits=[.1, .1]
j_limits=[1, 1]

traj_vscale=0.8    # can get osc when >.61
traj_ascale=0.1

# ruckig bits
inp = InputParameter(2)
otg = Ruckig(2, ts)
out = OutputParameter(2)

# define a trajectory
maxpoints=5
ptraj=[]
vtraj=[]
atraj=[]
for i in range(maxpoints):
    ang=i/5
    #ptraj.append([sin(ang+0), sin(ang+1)])
    ptraj.append([sin(ang+0), sin(ang+1)])
    #vtraj.append([0.8*v_limits[0]*cos(ang+0), 0.8*v_limits[1]*cos(ang+1)])
    vtraj.append([traj_vscale*v_limits[0]*cos(ang+0), traj_vscale*v_limits[1]*cos(ang+1)])
    #atraj.append([-0.8*a_limits[0]*sin(ang+0), -0.8*a_limits[1]*sin(ang+1)])
    atraj.append([-traj_vscale*a_limits[0]*sin(ang+0), -traj_vscale*a_limits[1]*sin(ang+1)])

# define initial robot position
inp.current_position = [0,0]#ptraj[0]
inp.current_velocity = [0,0]#vtraj[0]
inp.current_acceleration = [0,0]#atraj[0]

# define robot limits
inp.max_velocity = v_limits
inp.max_acceleration = a_limits
inp.max_jerk = j_limits

# define ruckig config params
# inp.synchronization= Synchronization.Phase


#current trajectory point
current_point=1

# for plotting
pos=[]
goal_pos=[]
vel=[]
goal_vel=[]
acc=[]
goal_acc=[]
time=[0]    # yay off by one error!
while current_point<maxpoints:
    inp.target_position = ptraj[current_point]
    inp.target_velocity = vtraj[current_point]
    inp.target_acceleration = atraj[current_point]

    # for plotting
    pos.append(inp.current_position)
    goal_pos.append(inp.target_position)
    vel.append(inp.current_velocity)
    goal_vel.append(inp.target_velocity)
    acc.append(inp.current_acceleration)
    goal_acc.append(inp.target_acceleration)
    time.append(time[-1]+ts)

    # call ruckig
    res = otg.update(inp, out)

    inp.current_position = out.new_position
    inp.current_velocity = out.new_velocity
    inp.current_acceleration = out.new_acceleration

    if res==Result.Finished:
        current_point=current_point+1

time.pop()  # fix the off by one error we introduced earlier.
print("done, now plotting")

# axis=0
# gstep=ptraj[1][0]-ptraj[0][axis]
# vinit=vtraj[0][axis]
# print(f"1: goal step: {gstep}, v init: {vinit}, expected time: {gstep/vinit}")
# axis=1
# gstep=ptraj[1][0]-ptraj[0][axis]
# vinit=vtraj[0][axis]
# print(f"2: goal step: {gstep}, v init: {vinit}, expected time: {gstep/vinit}")
# axis=0

import matplotlib.pyplot as plt
from numpy import array
for axis in [0,1]:
    plt.subplot(3,1,1)
    plt.plot(time, array(pos).transpose()[axis], label="Ruckig position")
    plt.plot(time,array(goal_pos).transpose()[axis], label="Target position")
    plt.legend()

    plt.subplot(3,1,2)
    plt.plot(time,array(vel).transpose()[axis], label="Ruckig velocity")
    plt.plot(time,array(goal_vel).transpose()[axis], label="Target velocity")
    plt.subplot(3,1,3)
    plt.plot(time,array(acc).transpose()[axis], label="Ruckig acceleration")
    plt.plot(time,array(goal_acc).transpose()[axis], label="Target acceleration")
    #plt.show()
    plt.savefig("axis-"+str(axis+1)+".png")
    plt.close()
