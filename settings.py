import pygame

def init():
    global pixels_per_meter
    global grid_width
    global grid_height
    global obstacle_height 
    global screen
    global car_length
    global car_width
    global trailer_length
    global trailer_width
    global trailer_gap
    global time_step
    global sim_time
    global max_turning_angle

    pixels_per_meter = 4 # 4 pixels per meter
    grid_width = 250 # 250 meters
    grid_height = 250 # 250 meters
    obstacle_height = 5 # 5 meter
    car_length = 6 # 10 meters
    car_width = 3 # 5 meters
    trailer_length = 3 # 5 meters
    trailer_width = 3 # 5 meters
    trailer_gap = 1 # 1 meter
    time_step = 0.1 # 0.1 seconds
    sim_time = 0.2 # 0.5 seconds
    max_turning_angle = 40 # 40 degrees
    pygame.init()
    screen = pygame.display.set_mode((grid_width * pixels_per_meter, grid_height * pixels_per_meter))
    pygame.display.set_caption("Multi_truck_planning")
    screen.fill((93,83,79))


