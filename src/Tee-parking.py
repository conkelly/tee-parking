#!/usr/bin/env python 
import roslib
import rospy 
import numpy as np 
import math

#This is supposed to be the code for a robot leading an airplane into a tee-hangar


#import ros nodes, unsure what they'll be. 


#We have that a Tee-Hangar is essentially of the form
#
#                             |                  |
#                             |                  |...We don't have this length but let's assume it's 4m just for ease.
#                             |                  |
#         ____________________|..... 4 m ........|_____________________
#        |                                                            |
#        4m                                                           |
#        |                                                            |
#        |___________________________19 m_____________________________|
#
#
#   My plan is to neglect the large wings and simply have a large hitbox and the position of the larger wings will dictate where the plane 
# is described to be within the hitbox. This also dictates finding the center point of the parking space.
# So this is simply a parking algorithm of something of the form where we are not permitted to go over the lines marked |.
#
#                          |            |
#                          8 m          |
#                          |            |
#                          |....4 m ....|
#


#For our purposes in this initial step I will be modeling an airplane as a cylender of marginal shorter distance, e, than the width of 
#the narrowest point of the barrack and length than the total of the barrack. The idea being that any plane will be contained 
#within these "hit boxes" and friction is arbitrary. Initially we are assuming that the margin is so low that our hitbox being contained in this 
# Our program ignores height for now. 

margin = 0.5
#overall length of hitbox
T_large_width = 19
T_short_width = 4
T_transition_length = 4
T_total_length = 8

V_long_wing_span = T_large_width - 2*margin
V_short_wing_span = T_short_width - 2*margin 
V_body_span = T_transition_length - 4*margin
V_total_length = T_total_length - 2*margin

#Unsure on how to quantify turning but let's assume some arbitrary values that we can rectify later

wheel_base = 12
overhang = (V_total_length - wheel_base)/2
max_steering_angle = 0.6 #Radians
minimum_turning_radius = 5 #meters
inner_turning_radius = np.sqrt(minimum_training_radius**2 - wheel_base**2) - (V_body_span/2)
outer_turning_radius = np.sqrt((inner_turning_radius + V_body_span)**2 + (wheel_base + overhang)**2)
min_depth = overhang + np.sqrt(outer_turning_radius**2 - inner_turning_radius**2)

def main(): 
    #For starters we will set it so that it is constant. 
    #Not sure about drifting or correcting such problems. 
    if V_short_wing_span <= T_short_width and V_long_wing_span <= T_large_width and T_total >= V_total_length: 
        rospy.init_node('Tee-parking')
        #subscribe to sensors on the ground. Unsure how this will work. 
        rospy.spin()
    else: 
        print("Not enough room")

def d(point_a, point_b): 
    return np.sqrt((point_a[0]-point_b[0])**2 + (point_a[1] - point_b[1])**2)

def park_points(ri, current, goal, w):
    r_prime = ri + w/2
    c1 = np.array([goal[0], goal[1] + r_prime])

    x_c1 = c1[0]
    y_c1 = c1[1]

    x_i = current[0]
    y_i = current[1]

    y_s = y_i
    y_c2 = y_s - r_prime

    y_t = (y_c1 + y_c2)/2
    x_t = x_c1 + np.sqrt(r_prime**2 - (y_t - y_c1)**2)

    x_s = 2 * x_t - x_c1

    x_c2 = x_s

    c2 = np.array([x_c2, y_c2])
    i = np.array([x_i, y_i])
    s = np.array([x_s, y_s])
    pt = np.array([x_t, y_t])
    #print r_prime, c1, c2, i, s, pt
    return r_prime, c1, c2, i, s, pt


def veh_control(initial_pos, start_pos, goal_pos, update_pos, orientation): 
    #I could use an AI search algorithm here but that works on discrete data, not continuous.
    #allign plane with center of initial opening
    #Take orientation as to mean -1 is looking in the negative direction, i to looking towards the goal position. 
    #Unsure how I would actually check so at this point this is mostly pseudo.
    #We have no knowledge of the obstacles on the other side and no sensors so 
    
    #We have that the back of the plane is the front of the robot. 
    #I'm treating the size of the robot as negligeable.
    global V_long_wing_span
    if update_pos[1] + V_long_wing_span / 2 < start_pos[1]: 
        if orientation == -i: 
            return 12, 0
        else:
            return 0, 5 #see concerns below. We just need to check if there's enough room first 
            #and we are in the same latteral position.
 
    elif update_pos[0] > start_pos[0]: 
        if orientation == -1: 
            return 12, 0
        else: 
            return 0, 5 #if it's facing the wrong way, turn around. 
            #Unsure if I'm turning in the right direction. Want to gaurantee there's enough room.
    elif update_pos[0] < start_pos[0]: 
        if orientation == 1: 
            return 12, 0
        else: return 0, -5 #Same concerns as previous comment.
    elif update_pos[0] == start_pos[0] and update_pos[1] < goal_pos[1] #Here we take by default the start_pos[1] > goal_pos[1] unsure what it would look like in a model. 
        if orientation == i: 
            return 12, 0 
        else: return 0, 5 
        #This function returns velocity, steering. 
    #Unsure how the robot would be empowered to correct for wobble

def veh_mission(msg): 
    #Set up publishers. Unsure what these will be called yet or what inputs the robot would have
    rate = rospy.Rate(100)
    #Set up obstacles as dimensions of inner wall planes. 
    #Unsure how to define goal position but we can set that up as a function once I understand the paramaters better 
    goal = np.array([0,0])
    launch_coord = np.array([20, 20])
    current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
    r_star, center1, center2, initial, start, transition = park_points(inner_turning_radius, launch_coord, goal, width)
    vel, steer = veh_control(initial, start, transition, goal, r_star, current_pos)

    #publish 
if __name__=="__main__": 
    main()
