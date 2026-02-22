#Create an environment for multi_trailer_truck planning. In the environment there are pygame rectangles which are obstacles.

import settings
import numpy as np
import pygame
import utils
import math
import sys

class environment():
    def __init__(self):
        self.obstacles = []
        self.obstacle_height = settings.obstacle_height
        self.grid_width = settings.grid_width
        self.grid_height = settings.grid_height
        self.pixels_per_meter = settings.pixels_per_meter
        self.screen = settings.screen
        self.obstacle_color = (0, 0, 0)
        self.car_color = (10, 20, 150)
        self.obstacle_width = 3
        self.obstacle_height = 3
        self.car_obstacle_height = 7
        self.car_obstacle_width = 10
        self.square_obstacle_height = 10
        self.square_obstacle_width = 10
        self.square_color = (150, 0, 0)

    
    def create_obstacles(self):
        # for i in range(10):
        #     self.obstacles.append(pygame.Rect((self.obstacle_height*(i+1)-self.obstacle_height/2)*self.pixels_per_meter, (200 - self.obstacle_width/2)*self.pixels_per_meter\
        #                                     , self.obstacle_height*self.pixels_per_meter, self.obstacle_width*self.pixels_per_meter))
            
        # for i in range(10):
        
        #     self.obstacles.append(pygame.Rect((150 + self.obstacle_height*(i+1)-self.obstacle_height/2)*self.pixels_per_meter, (150 - self.obstacle_width/2)*self.pixels_per_meter\
        #                                     , self.obstacle_height*self.pixels_per_meter, self.obstacle_width*self.pixels_per_meter))
            
        # for i in range(10):
        #     self.obstacles.append(pygame.Rect(165*settings.pixels_per_meter, (75 + self.obstacle_width*(i+1))*self.pixels_per_meter\
        #                                     , self.obstacle_height*self.pixels_per_meter, self.obstacle_width*self.pixels_per_meter))
            
        # for i in range(10):
        #     self.obstacles.append(pygame.Rect(185*settings.pixels_per_meter, (75 + self.obstacle_width*(i+1))*self.pixels_per_meter\
        #                                     , self.obstacle_height*self.pixels_per_meter, self.obstacle_width*self.pixels_per_meter))
        
        #self.obstacles.append(pygame.Rect((125-self.obstacle_height/2)*self.pixels_per_meter, (15 - self.obstacle_width/2)*self.pixels_per_meter\
        #                                    , self.obstacle_height*self.pixels_per_meter, self.obstacle_width*self.pixels_per_meter))

        for i in range(60):
            self.obstacles.append(pygame.Rect(70*settings.pixels_per_meter, (self.obstacle_width*(i+1))*self.pixels_per_meter,\
                                            self.obstacle_height*self.pixels_per_meter, self.obstacle_width*self.pixels_per_meter))
            
        for i in range(60):
            self.obstacles.append(pygame.Rect(160*settings.pixels_per_meter, (60 + self.obstacle_width*(i+1))*self.pixels_per_meter,\
                                            self.obstacle_height*self.pixels_per_meter, self.obstacle_width*self.pixels_per_meter))
            
        # adding car obstacles
        self.obstacles.append(pygame.Rect(50*self.pixels_per_meter, 50*self.pixels_per_meter\
                                            , self.car_obstacle_height*self.pixels_per_meter, self.car_obstacle_width*self.pixels_per_meter))
        self.obstacles.append(pygame.Rect(50*self.pixels_per_meter, 80*self.pixels_per_meter\
                                            , self.car_obstacle_height*self.pixels_per_meter, self.car_obstacle_width*self.pixels_per_meter))
        
        self.obstacles.append(pygame.Rect(50*self.pixels_per_meter, 110*self.pixels_per_meter\
                                            , self.car_obstacle_height*self.pixels_per_meter, self.car_obstacle_width*self.pixels_per_meter))
        
        self.obstacles.append(pygame.Rect(170*self.pixels_per_meter, 110*self.pixels_per_meter\
                                            , self.car_obstacle_height*self.pixels_per_meter, self.car_obstacle_width*self.pixels_per_meter))
        
        self.obstacles.append(pygame.Rect(170*self.pixels_per_meter, 130*self.pixels_per_meter\
                                            , self.car_obstacle_height*self.pixels_per_meter, self.car_obstacle_width*self.pixels_per_meter))
        
        # square obstacle 
        self.obstacles.append(pygame.Rect(115*self.pixels_per_meter, 100*self.pixels_per_meter\
                                            , self.square_obstacle_height*self.pixels_per_meter, self.square_obstacle_width*self.pixels_per_meter))
        
        
        





        

    def draw_obstacles(self):
        # for obstacle in self.obstacles:
        #     pygame.draw.rect(self.screen, self.obstacle_color, obstacle)
        for i in range(120):
            pygame.draw.rect(self.screen, self.obstacle_color, self.obstacles[i])

        for i in range(120,125):
            pygame.draw.rect(self.screen, self.car_color, self.obstacles[i])

        pygame.draw.rect(self.screen, self.square_color, self.obstacles[125])

            


# Class to define the multi_trailer_truck whos path we want to plan. The truck is defined by its position and orientation.
class truck_trailer():
    def __init__(self, num_trailers):
        car = pygame.image.load("truck.png")
        self.car = pygame.transform.scale(car, (int(settings.car_length*settings.pixels_per_meter), int(settings.car_width*settings.pixels_per_meter)))
        self.car_rect,self.car_image = utils.blitRotate(car, (0, 0), (car.get_width()/2, car.get_height()/2), 0)
        self.num_trailers = num_trailers

        self.trailer_images = []
        self.trailer_rects = []

        self.trailer_images_moved = [None for i in range(num_trailers)]
        self.trailer_rects_moved = [None for i in range(num_trailers)]

        #create the trailers. Each trailer is a pygame rectangle. Each should be behind the other. First trailer is closest to the truck.
        for _ in range(num_trailers):
            trailer = pygame.image.load("trailer.png").convert_alpha()
            trailer = pygame.transform.scale(trailer, (int(settings.trailer_length*settings.pixels_per_meter), int(settings.trailer_width*settings.pixels_per_meter)))
            trailer_rect,trailer_image = utils.blitRotate(trailer, (0, 0), (trailer.get_width()/2, trailer.get_height()/2), 0)
            self.trailer_images.append(trailer_image)
            self.trailer_rects.append(trailer_rect)

    def move_truck(self,position,orientation):

        self.car_rect,self.car_image = utils.blitRotate(self.car, (position[0]*settings.pixels_per_meter, settings.grid_height*settings.pixels_per_meter - position[1]*settings.pixels_per_meter), (self.car.get_width()/2, self.car.get_height()/2), orientation[0])
        for i in range(len(self.trailer_images)):
            if i == 0:
                position = (int(position[0] - int((settings.car_length/2 + settings.trailer_gap)*math.cos(math.radians(orientation[i])))), position[1] - int((settings.car_length/2 + settings.trailer_gap)*math.sin(math.radians(orientation[i]))))
                self.trailer_rects_moved[i],self.trailer_images_moved[i] = utils.blitRotate(self.trailer_images[i], (position[0]*settings.pixels_per_meter, settings.grid_height*settings.pixels_per_meter - position[1]*settings.pixels_per_meter), (self.trailer_images[i].get_width() + settings.trailer_gap*settings.pixels_per_meter, self.trailer_images[i].get_height()/2), orientation[i+1])
               
            else:
                position = (int(position[0] - int((settings.trailer_length + settings.trailer_gap)*math.cos(math.radians(orientation[i])))), position[1] - int((settings.trailer_length + settings.trailer_gap)*math.sin(math.radians(orientation[i]))))
                self.trailer_rects_moved[i],self.trailer_images_moved[i] = utils.blitRotate(self.trailer_images[i], (position[0]*settings.pixels_per_meter, settings.grid_height*settings.pixels_per_meter - position[1]*settings.pixels_per_meter), (self.trailer_images[i].get_width() + settings.trailer_gap*settings.pixels_per_meter, self.trailer_images[i].get_height()/2), orientation[i+1])



    #orientation is the angle in degrees, for the truck and each trailer, it is a list. The orientation of each trailer is relative to the truck. The truck's orientation is global.
    def draw_truck(self, screen, position, orientation):

        self.move_truck(position, orientation)
        screen.blit(self.car_image, self.car_rect)
        for i in range(len(self.trailer_images)):
            screen.blit(self.trailer_images_moved[i], self.trailer_rects_moved[i])

        

    def check_collision(self,env, self_check = False):
        for obstacle in env.obstacles:
            if self.car_rect.colliderect(obstacle): #check if the truck collides with any obstacle
                return True
            for trailer in self.trailer_rects_moved:
                if trailer.colliderect(obstacle):
                    return True
                
                
        # self collision check
        if self_check:
            for i in range(len(self.trailer_rects_moved)):
                for j in range(i+1, len(self.trailer_rects_moved)):
                    if self.trailer_rects_moved[i].colliderect(self.trailer_rects_moved[j]):
        
                        return True
                    
            for i in range(len(self.trailer_rects_moved)):
                if self.trailer_rects_moved[i].colliderect(self.car_rect):
            
                    return True
        return False



if __name__ == "__main__":
    settings.init()
    env = environment()
    auto_truck = truck_trailer(3)
    auto_truck.draw_truck(env.screen, (125, 100), [0,10,0,0])
    env.create_obstacles()
    #env.draw_obstacles()
    pygame.display.update()
    print(auto_truck.check_collision(env))
    i = 0
    x = 125
    y = 125
    theta = [45,55,65,45]
    for i in range(len(theta)):
        theta[i] = math.radians(theta[i])
    velocity = 10
    steering_angle = 0
    while True:
        i +=1
        print(sys.getsizeof(auto_truck),i)
        # get random position and orientation
        x += 1.0*math.cos((theta[0]))
        y += 1.0*math.sin((theta[0]))
        #theta = [theta[0] + 10, theta[1], theta[2], theta[3]]
       # theta = [int(i) for i in theta]
        theta[0] += velocity * math.tan(math.radians(steering_angle))*settings.time_step / settings.car_length
            # wrap the angle between - pi and pi
        theta[0] = (theta[0] + np.pi) % (2 * np.pi) - np.pi
        
      
        theta[1] = theta[1] + settings.time_step*velocity * math.sin((theta[0] - theta[1])) / settings.trailer_length
        theta[1] = (theta[1] + np.pi) % (2 * np.pi) - np.pi

     
        for i in range(2,4):
            # lambda function to calculate the product of cosines of all differences of successive angles till angle i, that is product of cos(theta[0] - theta[1]) * cos(theta[1] - theta[2]) * ... * cos(theta[i-2] - theta[i-1])
            product_cos = lambda i: np.prod(np.cos((np.diff(theta[:i-1]))))
            theta[i] = theta[i] + settings.time_step*velocity *product_cos(i)* math.sin((theta[i-1] - theta[i])) / settings.trailer_length
            theta[i] = (theta[i] + np.pi) % (2 * np.pi) - np.pi 

        theta_deg = [math.degrees(i) for i in theta]

        auto_truck.draw_truck(env.screen, (x,y),theta_deg)
        pygame.display.flip()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        
        pygame.time.wait(100)
        settings.screen.fill((255, 255, 255))
        pass