import pygame
import math
import time

WIDTH = 800
HEIGHT = 480
MAIN_COLOR = (0, 155, 195)
BLACK = (0, 0, 0)
FPS = 60

def start_ui():
    start_time = time.monotonic()
    pygame.init()
    # Set up the drawing window
    screen = pygame.display.set_mode([WIDTH, HEIGHT])
    # Run until the user asks to quit
    running = True
    offset = 0
    while running:
        screen.fill(BLACK)
        # Did the user click the window close button?
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Fill the background with white

        # Draw a solid blue circle in the center
        circle_center = (250, HEIGHT+180)
        circle_radius = 240
        pygame.draw.circle(screen, MAIN_COLOR, circle_center, circle_radius, width=3)
        for i in range(100):
            angle = (offset + i)*math.pi/50
            new_radius = circle_radius - 2
            vec = (new_radius*math.sin(angle), new_radius*math.cos(angle))
            tic_length = .95
            if i % 10 == 0:
                tic_length = .9
            vec2 = (vec[0]*tic_length, vec[1]*tic_length)
            end = (circle_center[0] + vec[0], circle_center[1] + vec[1])
            end2 = (circle_center[0] + vec2[0], circle_center[1] + vec2[1])
            pygame.draw.line(screen, MAIN_COLOR, end2, end, width=2)

        font = pygame.font.SysFont(None, 24)
        img = font.render(f'{offset}', True, MAIN_COLOR)
        screen.blit(img, (circle_center[0], circle_center[1]-255))

        offset = int(time.monotonic() - start_time)
        # Flip the display
        pygame.display.flip()

    # Done! Time to quit.
    pygame.quit()