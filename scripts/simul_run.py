import time
import numpy as np
import pygame
import yaml
import copy

from utils.utils import *
from utils.color import COLOR
from utils.object_simul import ObjectDragger, ObjectPullee, ObjectObstacle
from utils.drag_server import DragServer

def create_background_surface():
    # Create a background_surface surface
    background_surface = pygame.Surface((WIDTH, HEIGHT))
    background_surface.fill(WHITE)

    # Draw gridlines
    # 0.04m spaing
    gap = 0.04 / unit     # Guideline lengh
    for y_idx in range(int(HEIGHT / gap)): pygame.draw.line(background_surface, LIGHTGRAY, (0, y_idx * gap), (WIDTH, y_idx * gap), 2)  # horizontal gridlines
    for x_idx in range(int(WIDTH  / gap)): pygame.draw.line(background_surface, LIGHTGRAY, (x_idx * gap, 0), (x_idx * gap, HEIGHT), 2) # vertical gridlines
    # 0.2m spacing
    gap = 0.2 / unit      # Guideline lengh
    for y_idx in range(int(HEIGHT / gap)): pygame.draw.line(background_surface, DARKGRAY, (0, y_idx * gap), (WIDTH, y_idx * gap), 2)   # horizontal gridlines
    for x_idx in range(int(WIDTH  / gap)): pygame.draw.line(background_surface, DARKGRAY, (x_idx * gap, 0), (x_idx * gap, HEIGHT), 2)  # vertical gridlines
    return background_surface

def create_polygon_surface(object, color):
    offset_x, offset_y = WIDTH / 2, HEIGHT / 2
    if isinstance(object, ObjectDragger):
        # Convert polygon points coordinate to pygame display coordinate
        x, y, _ = object.q / unit
        r       = object.r / unit
        x += offset_x
        y *= -1
        y += offset_y
        circle_surface = pygame.Surface((2*r, 2*r), pygame.SRCALPHA)
        circle_surface.fill((0, 0, 0, 0))  # Transparent background
        pygame.draw.circle(circle_surface, BLUE, (r,r), r)
        pygame.draw.line(circle_surface, LIGHTGRAY, (r * 2 / 4, r), (r * 6 / 4, r), 2)   # Draw horizontal line
        pygame.draw.line(circle_surface, LIGHTGRAY, (r, r * 2 / 4), (r, r * 6 / 4), 2)   # Draw vertical line
        rotated_surface = pygame.transform.rotate(circle_surface, np.rad2deg(object.q[-1]))
        circle_rect = rotated_surface.get_rect(center=(x, y))
        # circle_rect = circle_surface.get_rect(center=(x, y))
        screen.blit(rotated_surface, circle_rect.topleft)
        
    elif isinstance(object, ObjectPullee):
        # Convert polygon points coordinate to pygame display coordinate
        w, h    = object.width / unit, object.height / unit
        x, y, _ = object.q / unit
        theta   = np.rad2deg(object.q[-1])
        x += offset_x
        y *= -1
        y += offset_y
        rect_surface = pygame.Surface((w, h), pygame.SRCALPHA)
        pygame.draw.rect(rect_surface, RED, (0, 0, w, h))
        pygame.draw.line(rect_surface, LIGHTGRAY, (w / 4, h / 2), (w * 3 / 4, h / 2), 2)   # Draw horizontal line
        pygame.draw.line(rect_surface, LIGHTGRAY, (w / 2, h / 4), (w / 2, h * 3/ 4), 2)   # Draw vertical line
        rotated_surface = pygame.transform.rotate(rect_surface, theta)
        rotated_rect = rotated_surface.get_rect(center=(x, y))
        screen.blit(rotated_surface, rotated_rect.topleft)
    elif isinstance(object, ObjectObstacle):
        pass
    else:
        raise ValueError('Invalid object type')

def create_force_interface():
    input_text = font.render(f"Input: {input_number}", True, BLACK)
    screen.blit(input_text, (50, 75))
    pygame.draw.rect(screen, LIGHTGRAY, button_rect)
    button_text = font.render("Force", True, BLACK)
    text_rect = button_text.get_rect(center=button_rect.center)
    screen.blit(button_text, text_rect)

def create_pose_interface(dragger, pullee):
    pass

###############################
### Simulation setting code ###
###############################

### Get the simulation setting from the yaml file
with open('../config/config.yaml', 'r') as f:
    config = yaml.load(f, Loader=yaml.FullLoader)

## Set pygame display
# Set display parameters
WIDTH, HEIGHT = config['display']['WIDTH'], config['display']['HEIGHT']
display_center = np.array([WIDTH / 2, HEIGHT / 2])

# Set display color
WHITE   = COLOR['WHITE']
BLACK   = COLOR['BLACK']
RED     = COLOR['RED']  
GREEN   = COLOR['GREEN']
BLUE    = COLOR['BLUE']
YELLOW  = COLOR['YELLOW']
LIGHTGRAY = COLOR['LIGHTGRAY']
DARKGRAY  = COLOR['DARKGRAY']

# Set pixel unit
unit = config['display']['unit']

## Set simulation setting
# Init dragger pos and rot
dragger_position = config['dragger']['init_position']
dragger_rotation = config['dragger']['init_rotation']
contact_force    = config['dragger']['contact_force']
contact_radius   = config['dragger']['contact_radius']

# Init pullee pos and rot
pullee_position = config['pullee']['init_position']
pullee_rotation = config['pullee']['init_rotation']
pulllee_width   = config['pullee']['WIDTH']
pullee_height   = config['pullee']['HEIGHT']

# Init obstacles info
obstacles_info  = config['obstacles']

# Set control speed
unit_v_speed    = config['dragger']['unit_v_speed']
unit_r_speed    = config['dragger']['unit_r_speed']

# Set simulator fps & sim-step
fps = config['simulator']['fps']
sim_step = config['simulator']['sim_step']

## Generate objects
# Generate Dragger
dragger = ObjectDragger(dragger_position, dragger_rotation, contact_radius, contact_force)
# Generate Pullee
pullee = ObjectPullee(pullee_position, pullee_rotation, pulllee_width, pullee_height)
# Generate Obstacle
obstacles = ObjectObstacle(obstacles_info)

pygame.init()
font = pygame.font.Font(None, 36)
pygame.display.set_caption('Top-Contact-Dragging Simulation')
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()
background = create_background_surface()
button_rect = pygame.Rect(75, 125, 100, 50)
input_number = str(contact_force)
u_input = np.zeros(3)

drag_server = DragServer()

# Main loop 
while True:
    #############################
    # Step1. Keyboard interaction
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            break
        # Keyboard input for force value
        elif event.type == pygame.KEYDOWN:
            if event.unicode.isdigit():  
                input_number += event.unicode
            elif event.unicode == "." and "." not in input_number:  
                input_number += event.unicode
        # Mouse input for pressing button
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if button_rect.collidepoint(event.pos):
                try:
                    contact_force = float(input_number)                    
                    print('Force Value Adjusted: ' + input_number)
                    dragger.N = contact_force
                    input_number = ""
                except ValueError:
                    print('Invalid input number')
                    
    # Move dragging point
    keys = pygame.key.get_pressed()
    # Finish simulator (esc)
    if keys[pygame.K_ESCAPE]: break                         
    # Move pusher center in y-axis (ws)
    if keys[pygame.K_w]: u_input[1]     = unit_v_speed
    elif keys[pygame.K_s]: u_input[1]   = -unit_v_speed
    else:                u_input[1]     = 0.0
    # Move pusher center in x-axis (ad)
    if keys[pygame.K_a]: u_input[0]     = -unit_v_speed 
    elif keys[pygame.K_d]: u_input[0]   = unit_v_speed
    else:                u_input[0]     = 0.0
    # Rotate pusher (qe)
    if keys[pygame.K_q]: u_input[2]     = unit_r_speed
    elif keys[pygame.K_e]: u_input[2]   = -unit_r_speed
    else:                u_input[2]     = 0.0
    dragger.v = u_input
    
    #############################
    # Step2. Update parameters & Run simulation
    derived_vel = np.zeros(3)
    if is_circle_inside_rotated_rectangle(dragger, pullee):
        drag_server.update(dragger, pullee)
        derived_vel = drag_server.object_velocity_calculation(u_input)
    else:
        pass

    #############################
    # Step3. Apply simulation results
    dragger.apply_v(u_input, sim_step)
    pullee.apply_v(derived_vel, sim_step)

    #############################
    # Step4. Visualization
    # Update pygame display
    # Bliting background
    screen.blit(background, (0, 0))
    # Bliting Pullee
    pullee_surface = create_polygon_surface(pullee, GREEN)
    # Bliting Dragger
    dragger_surface = create_polygon_surface(dragger, RED)
    # Bliting Obstacle
    obstacles_surface = create_polygon_surface(obstacles, BLUE)
    # Bliting Input text
    create_force_interface()
    # Bliting Input text
    create_pose_interface(dragger, pullee)
    # Update the display
    pygame.display.flip()
    clock.tick(fps)

