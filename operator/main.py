import asyncio
import pygame
import keyboard
import websockets
import json

# pygame setup

pygame.init()
'''
control scheme:
left stick: strafe
right stick: rotate
'''
joystick = pygame.joystick.Joystick(0)
# rightStick = pygame.joystick.Joystick(1)
pygame.joystick.init()

height = 300
width = 400
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()
running = True
ip = "192.168.1.61"

def UI(left, right):
    pygame.draw.line(screen,"red",pygame.Vector2(width/2 - 50,height/2), pygame.Vector2(width/2 - 50, left * 100 + height/2), 25)
    pygame.draw.line(screen,"blue",pygame.Vector2(width/2 + 50,height/2), pygame.Vector2(width/2 + 50, right * 100 + height/2), 25)

async def main():
    running = True
    leftOld = joystick.get_axis(1)
    rightOld = joystick.get_axis(3)
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                # await quit(motionQueue)
                # drive_task.cancel()
                # servo_task.cancel()
                running = False
                break

        screen.fill((0, 0, 0))
        UI(joystick.get_axis(1),joystick.get_axis(3))
        pygame.display.flip()
        
asyncio.run(main())
pygame.quit()