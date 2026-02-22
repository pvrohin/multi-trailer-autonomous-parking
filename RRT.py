# using RRT planning algorithm for multi_trailer_truck path planning

import pygame
import math
import numpy as np
import reeds_shepp
import settings
import A_star
import scipy.stats
import environment
import time


# set seed

np.random.seed(42)


#Class to define the node in the graph
class RRTNode():
    def __init__(self, x, y, orientations):
        self.x = x
        self.y = y
        self.orientations = orientations
        self.parent = None
        self.cost_to_come = 0
        self.cost = 0
        self.node_goal_weight = 0 # weight of a node, if it is closest to the goal node in the nearest neighbour search. After this node is selected and the path is generated,
        # if the generated path leads to collision, then the cost of this node is increased by this weight. This is to ensure that the path is not too close to the obstacles. 
        # This cost is incorporated in nearest neighbour search, if the node whose nearest neighbours are being searched is the goal node.

    # comparing costs of two nodes
    def __lt__(self, other):
        return self.cost < other.cost
    
    def __gt__(self, other):
        return self.cost > other.cost
    
    def __le__(self, other):
        return self.cost <= other.cost
    
    # To retrun that the two nodes are equal if they have the same int(x), int(y) and orientation is similar to within 5 degrees
    def __eq__(self, other):

        if other != None:
            return int(self.x) == int(other.x) and int(self.y) == int(other.y) and np.all(np.min([np.abs(self.orientations - other.orientations), 360 - np.abs(self.orientations - other.orientations)], axis = 0) < 60)
        else:
            return False

#Class to define the RRT algorithm
class RRT():
    def __init__(self, start, goal, world, truck_trailer):
        self.start_node = RRTNode(start[0], start[1], start[2])
        self.goal_node = RRTNode(goal[0], goal[1], goal[2])
        self.environment = world
        self.truck_trailer = truck_trailer
        self.num_trailers = self.truck_trailer.num_trailers
        self.forward_tree = [self.start_node]
        self.max_iter = 10000
        # This is the environment without obstacles. Used to generate paths for the truck and trailers quickly. This generated path is then checked for collision with the actual environment.
        self.free_environment = environment.environment()
        self.max_time = 900
        self.tolerance = 5


    def sample(self): # Random position and orientation, based on number of trailers

        # Keep some samples near the obstacles
        # With a small probability the goal node should be returned


        if np.random.random() < 0.1:
            print("selecting goal node")
            return self.goal_node
        
        else:
            sample_found = False
            random_number = np.random.random()
            if random_number< 0.2:
                
                while sample_found is not True:
                    print("selecting near goal node")
                    mux = self.goal_node.x
                    muy = self.goal_node.y
                    sigmax = settings.grid_width/10
                    sigmay = settings.grid_height/10
                    lower_limitx = (20 - mux)/sigmax
                    upper_limitx = (230 - mux)/sigmax
                    lower_limity = (20 - muy)/sigmay
                    upper_limity = (230 - muy)/sigmay
                    x = int(scipy.stats.truncnorm.rvs(lower_limitx,upper_limitx,loc = mux, scale = sigmax,size = 1))
                    y = int(scipy.stats.truncnorm.rvs(lower_limity,upper_limity,loc = muy, scale = sigmay,size = 1))

                    orientation_car = np.random.randint(self.goal_node.orientations[0] - 60, self.goal_node.orientations[0] + 60)
                    orientation_trailers = np.random.randint(orientation_car- 60, orientation_car + 60, self.num_trailers)
                    orientation_trailers = (orientation_trailers + 180) % (360) - 180
                    orientations = np.insert(orientation_trailers, 0, orientation_car)
                    self.truck_trailer.move_truck((x,y), orientations)
                    if not self.truck_trailer.check_collision(self.environment):
                        for node in self.forward_tree:
                            if node == RRTNode(x, y, orientations):
                                sample_found = False
                                break
                            else:
                                sample_found = True
                        if sample_found:
                            print(x,y,orientations)
                            return RRTNode(x, y, orientations)

            elif random_number < 0.4:
                
                while sample_found is not True:
                    print("selecting obstacle node")
                    i = np.random.randint(0, len(self.environment.obstacles))
                    obstacle_chosen = self.environment.obstacles[i]
                    mux = obstacle_chosen.centerx
                    muy = obstacle_chosen.centery
                    sigmax = settings.grid_width/15
                    sigmay = settings.grid_height/15
                    x = int(scipy.stats.truncnorm.rvs(20,230,loc = mux, scale = sigmax,size = 1))
                    y = int(scipy.stats.truncnorm.rvs(20,230,loc = muy, scale = sigmay,size = 1))
                    orientation_car = np.random.randint(0, 360)
                    orientation_trailers = np.random.randint(orientation_car- 60, orientation_car + 60, self.num_trailers)
                    orientation_trailers = (orientation_trailers + 180) % (360) - 180
                    orientations = np.insert(orientation_trailers, 0, orientation_car)
                    self.truck_trailer.move_truck((x,y), orientations)
                    if not self.truck_trailer.check_collision(self.environment):
                        for node in self.forward_tree:
                            if node == RRTNode(x, y, orientations):
                                sample_found = False
                                break
                            else:
                                sample_found = True
                        if sample_found:
                            return RRTNode(x, y, orientations)
                    
            else:
                
                while sample_found is not True:
                    print("selecting random node")
                    x = np.random.randint(0, self.environment.grid_width)
                    y = np.random.randint(0, self.environment.grid_height)
                    orientation_car = np.random.randint(0, 360)
                    orientation_trailers = np.random.randint(orientation_car- 60, orientation_car + 60, self.num_trailers)
                    orientation_trailers = np.mod(orientation_trailers, 360)
                    orientations = np.insert(orientation_trailers, 0, orientation_car)
                    self.truck_trailer.move_truck((x,y), orientations)
                    if not self.truck_trailer.check_collision(self.environment):
                        for node in self.forward_tree:
                            if node == RRTNode(x, y, orientations):
                                sample_found = False
                                break
                            else:
                                sample_found = True
                        if sample_found:
                            return RRTNode(x, y, orientations)

        # check if this sample is in collision

        return RRTNode(x, y, orientations)
    
    def nearest(self, node): # Find the nearest node in the tree to the sampled node
        min_dist = np.inf
        nearest_node = None

        if (node == self.goal_node):
            for tree_node in self.forward_tree:
                dist = reeds_shepp.path_length(np.array([node.x, node.y, node.orientations[0]*np.pi/180]), np.array([tree_node.x, tree_node.y, tree_node.orientations[0]*np.pi/180]), settings.car_length*2/math.tan(math.radians(settings.max_turning_angle)) )
                if dist < min_dist and tree_node.node_goal_weight == 0:
                    min_dist = dist
                    nearest_node = tree_node
            if nearest_node != None:
                return nearest_node
        
        for tree_node in self.forward_tree:
            dist = reeds_shepp.path_length(np.array([node.x, node.y, node.orientations[0]*np.pi/180]), np.array([tree_node.x, tree_node.y, tree_node.orientations[0]*np.pi/180]), settings.car_length*2/math.tan(math.radians(settings.max_turning_angle)) )
            if dist < min_dist:
                min_dist = dist
                nearest_node = tree_node

        return nearest_node
    
    def change_goal_weights(self, nodes): # change the weight variable to np.inf for half the nodes in the list
        nodes.reverse()
        for i in range(int(len(nodes)/2)):
            nodes[i].node_goal_weight = np.inf 
        nodes.reverse()
    
    def steer(self, nearest_node, sampled_node): 
        # Steer from nearest node to sampled node. Assuming no obstacles generate a path to the sampled node. Once path is generated add nodes to the tree 
        # as you are traversing the path, in some small increments. If obstacle is encountered, stop and return the last node added to the tree.
        # If the sampled node is within some small distance of the nearest node, return the sampled node.

        # First check if the sampled node is within some small distance of the nearest node
        if reeds_shepp.path_length(np.array([sampled_node.x, sampled_node.y, sampled_node.orientations[0]*np.pi/180]), np.array([nearest_node.x, nearest_node.y, nearest_node.orientations[0]*np.pi/180]), settings.car_length*4/math.tan(math.radians(settings.max_turning_angle)) ) < 5:
            return [sampled_node]
        
        # If the sampled node is not within some small distance of the nearest node, generate a path to the sampled node
        start = (nearest_node.x, nearest_node.y, nearest_node.orientations)
        goal =  (sampled_node.x, sampled_node.y, sampled_node.orientations)
        
        a_star_local = A_star.A_star(start, goal, self.free_environment, self.truck_trailer)

        nodes = []
        
        path = a_star_local.A_star_plan(time_limit=3)

        # Go through the path, at every point check if it collides with the actual environment. 
        # Add a node at every point, till either goal is checked or an obstacle is encountered
        # Return the list of nodes. Each nodes parent is the previous point in the path. 

        if path == None:
            return nodes
        
        nodes.append(nearest_node)
    
        
        for i in range(1,len(path)):
            self.truck_trailer.move_truck((path[i].x, path[i].y), path[i].orientations)
            if self.truck_trailer.check_collision(self.environment):
                if sampled_node == self.goal_node:
                    self.change_goal_weights(nodes)
                print("collision encountered")
                return nodes
            else:
                next_node = RRTNode(path[i].x, path[i].y, path[i].orientations)
                next_node.parent = nodes[-1]
                nodes.append(next_node)

        return nodes
    
    def dist_goal_tree(self, node, goal_node):
        # Find the euclidean distance of the node to the goal node
        return np.sqrt((node.x - goal_node.x)**2 + (node.y - goal_node.y)**2)
    
    def extract_path(self, node):
        # Extract the path from the goal node to the start node
        path = []
        while node != None:
            path.append(node)
            node = node.parent
        return path[::-1]
        
    def RRT_plan(self):
        # start the tree and keep on sampling, steering and adding nodes, till goal is reached.
        start_time = time.time()
        for _ in range(self.max_iter):
            if time.time() - start_time > self.max_time:
                print("Time limit exceeded")
                return None
            

            # if 1<2: #len(self.start_nodes) < len(self.end_nodes):
            qrand = self.sample()
            qnear = self.nearest(qrand)
            new_nodes = self.steer(qnear, qrand)
            print(" length of nodes and tree ",len(new_nodes), len(self.forward_tree))


            if len(new_nodes) != 0:
                del new_nodes[0]

            if len(new_nodes) != 0:
                # for node in new_nodes:
                self.forward_tree.extend(new_nodes)
                for node in new_nodes:
                    if self.dist_goal_tree(node,self.goal_node) < self.tolerance:
                        print("Path found in rrt")
                        nearest_node = self.nearest(self.goal_node)
                        return self.extract_path(nearest_node)
            # else:
            #     qrand = self.sample("end")
            #     qnear = self.nearest(self.end_nodes, qrand)
            #     new_nodes = self.steer(qnear, qrand.vertex)
            #     if new_nodes != None:
            #         for node in new_nodes:
            #             self.end_nodes.append(node)
            #             self.end_nodes_vertices.append(node.vertex)
            #         for node in new_nodes:
            #             if self.dist_goal_tree(node,self.start_nodes) < self.tolerance:
            #                 print("Path found")
            #                 nearest_node = self.nearest(self.start_nodes, node)
            #                 return self.extract_path(nearest_node, node)
        print("Path not found")
        return None
    
    def draw_tree(self):
        self.environment.screen.fill((255,255,255))

        for node in self.forward_tree: # draw lines from every node to its parent
            if node.parent != None:
                pygame.draw.line(self.environment.screen, (0,0,0), (int(node.x*settings.pixels_per_meter), int(settings.grid_height*settings.pixels_per_meter - node.y*settings.pixels_per_meter)), (int(node.parent.x*settings.pixels_per_meter), int(settings.grid_height*settings.pixels_per_meter - node.parent.y*settings.pixels_per_meter)), 1)
        pygame.display.flip()
        pygame.time.wait(10000)
    


if __name__  == "__main__":
    settings.init()
    env = environment.environment()
    truck_trailer = environment.truck_trailer(3)
    truck_trailer.draw_truck(env.screen, (0.0,0.0), (0.0,0.0,0.0, 0.0))
    env.create_obstacles()
    env.draw_obstacles()
    start = (20.0,150.0,np.array([-90.0,-90.0,-90.0,-90.0]))
    goal = (175.0,160.0,np.array([-90.0,-90.0,-90.0,-90.0]))
    pygame.draw.circle(env.screen, (0,0,255), (int(start[0]*settings.pixels_per_meter), int(settings.grid_height*settings.pixels_per_meter - start[1]*settings.pixels_per_meter)), 5)
    pygame.draw.circle(env.screen, (0,255,0), (int(goal[0]*settings.pixels_per_meter), int(settings.grid_height*settings.pixels_per_meter - goal[1]*settings.pixels_per_meter)), 5)
    pygame.display.flip()
    pygame.time.wait(1000)

    rrt = RRT(start, goal, env, truck_trailer)
    start_time = time.time()
    path = rrt.RRT_plan()
    print("Time taken for RRT planning is ", time.time() - start_time)

    env.screen.fill((93,83,79))
    env.draw_obstacles()
    pygame.display.flip()
    pygame.time.wait(8000)


    if path != None:
        for node in path:
            print(node.x, node.y, node.orientations)
            env.screen.fill((93,83,79))
            truck_trailer.draw_truck(env.screen, (node.x, node.y), node.orientations)
            env.draw_obstacles()
            pygame.display.flip()
            pygame.time.wait(100)
        

    rrt.draw_tree()

    pygame.quit()


