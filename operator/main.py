import asyncio
import zmq
import pygame

import cv2
import numpy as np

# pygame.init()
'''
control scheme:
left stick: strafe
right stick: rotate
'''
# joystick = pygame.joystick.Joystick(0)
# rightStick = pygame.joystick.Joystick(1)
# pygame.joystick.init()

height = 600
width = 800
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()
running = True

addr = "tcp://192.168.1.200:5555"
ctx = zmq.Context()
s = ctx.socket(zmq.SUB)
s.connect(addr)
s.setsockopt(zmq.SUBSCRIBE, b'')
print("Connected to address")

# def UI(left, right):
#     pygame.draw.line(screen,"red",pygame.Vector2(width/2 - 50,height/2), pygame.Vector2(width/2 - 50, left * 100 + height/2), 25)
#     pygame.draw.line(screen,"blue",pygame.Vector2(width/2 + 50,height/2), pygame.Vector2(width/2 + 50, right * 100 + height/2), 25)

def getFrame():
    buffer = s.recv()

    img_array = np.frombuffer(buffer, dtype=np.uint8)
    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

    if frame is not None:
        frame = np.rot90(frame)
        return frame
            
def frame2pix(frame):
    print("frame")
    frame = pygame.surfarray.make_surface(frame)
    screen.blit(frame, (0,0))

async def main():
    running = True
    # leftOld = joystick.get_axis(1)
    # rightOld = joystick.get_axis(3)
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break
        screen.fill((0, 0, 0))
        
        frame2pix(getFrame())
        # UI(joystick.get_axis(1),joystick.get_axis(3))
        pygame.display.flip()
    asyncio.sleep(0.1)        
asyncio.run(main())
pygame.quit()