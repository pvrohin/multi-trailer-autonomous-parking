# Utility functions for the project

#blit rotate function

import pygame
import math
import numpy as np
import reeds_shepp
import settings

def cost_to_go(current_node, goal_node, hueristic_check = False, environment = None):
    # current_node and goal_node are both instances of the Node class
    # returns the cost to go from current_node to goal_node
    # the cost to go is the sum of the reeds shepp path lengths for the truck and each trailer

    # final argument is the maximum turning radius of the truck
   
    
    # y = mx + c, m is current_node.orientations[0] passes through goal.x, goal.y
    # distance from this line is calculated for truck only
    
    trail_weight = 8/len(current_node.orientations)

    if hueristic_check:
        m = math.tan(math.radians(goal_node.orientations[0]))
        c = goal_node.y - m*goal_node.x
        distance = abs(m*current_node.x - current_node.y + c)/math.sqrt(m**2 + 1)
        if distance < 2 and (abs(current_node.orientations[0] - goal_node.orientations[0]) < 2):

            cost = 5 + 0.8*reeds_shepp.path_length(np.array([current_node.x,current_node.y,current_node.orientations[0]*np.pi/180]), np.array([goal_node.x, goal_node.y,goal_node.orientations[0]*np.pi/180]), settings.car_length*2/math.tan(math.radians(settings.max_turning_angle)) )
            trail_weight = 6/len(current_node.orientations)

        else:
            cost = reeds_shepp.path_length(np.array([current_node.x,current_node.y,current_node.orientations[0]*np.pi/180]), np.array([goal_node.x, goal_node.y,goal_node.orientations[0]*np.pi/180]), settings.car_length*2/math.tan(math.radians(settings.max_turning_angle)) )
            trail_weight = 4/len(current_node.orientations)

    else:
         cost = reeds_shepp.path_length(np.array([current_node.x,current_node.y,current_node.orientations[0]*np.pi/180]), np.array([goal_node.x, goal_node.y,goal_node.orientations[0]*np.pi/180]), settings.car_length*2/math.tan(math.radians(settings.max_turning_angle)) )

    trailer_turning_radius = settings.trailer_length/math.tan(math.radians(settings.max_turning_angle))

    
    if hueristic_check:
        for i in range(1,len(current_node.orientations)):
        
            cost += trail_weight*trailer_turning_radius*np.absolute(current_node.orientations[i] - current_node.orientations[i-1])*np.pi/180
        #print("cost3", trail_weight*trailer_turning_radius*np.absolute(current_node.orientations[i] - current_node.orientations[i-1]))

        # if environment != None:
        # # add cost based on distance from obstacles 1/(1+ exp(distance))
        #     for obstacle in environment.obstacles:
        #         distance = np.sqrt((current_node.x - obstacle.x)**2 + (current_node.y - obstacle.y)**2)
        #         #print(obstacle.x, obstacle.y, distance)
        #         cost += 1000/((1 + np.exp(distance/200))*len(environment.obstacles))
                #print("cost4", 2000/(1 + np.exp(distance/100)))
                #print("cost2", 1/(1 + np.exp(distance)))
                
    return cost


def blitRotate(image, pos, originPos, angle):
    image_rect = image.get_rect(topleft = (pos[0] - originPos[0], pos[1]-originPos[1]))
    offset_center_to_pivot = pygame.math.Vector2(pos) - image_rect.center
    rotated_offset = offset_center_to_pivot.rotate(-angle)
    rotated_image_center = (pos[0] - rotated_offset.x, pos[1] - rotated_offset.y)
    rotated_image = pygame.transform.rotate(image, angle)
    rotated_image_rect = rotated_image.get_rect(center = rotated_image_center)
    car_rect = rotated_image_rect
    car_image = rotated_image
    
    return car_rect, car_image


def generate_controls_matrix(velocity_controls, steering_controls):
	control_inputs_velocity, control_inputs_steering = np.meshgrid(velocity_controls, steering_controls, indexing='ij')

	control_inputs_velocity = np.ndarray.flatten(control_inputs_velocity)

	control_inputs_steering = np.ndarray.flatten(control_inputs_steering)

	control_inputs = list(zip(control_inputs_velocity,control_inputs_steering))

	control_inputs = np.array(control_inputs)

	return control_inputs