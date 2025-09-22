import keyboard
import time
import asyncio
import threading

import zmq
import pygame
import cv2
import numpy as np

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


running = True

addr = "tcp://100.121.88.110:"
ctx = zmq.Context()
commsReq = ctx.socket(zmq.REQ)
processRep = ctx.socket(zmq.REP)
sensorSub = ctx.socket(zmq.SUB)
videoSub = ctx.socket(zmq.SUB)

processRep.connect(addr + "5557")
videoSub.connect(addr + "5555")
videoSub.setsockopt(zmq.SUBSCRIBE, b'')
commsReq.connect("tcp://100.121.88.110:5556")

# print("test")
# commsReq.send(b"READY")
# print("READY sent. awaiting messge")
# ready = commsReq.recv()
# print("Connected to address")

# def UI(left, right):
#     pygame.draw.line(screen,"red",pygame.Vector2(width/2 - 50,height/2), pygame.Vector2(width/2 - 50, left * 100 + height/2), 25)
#     pygame.draw.line(screen,"blue",pygame.Vector2(width/2 + 50,height/2), pygame.Vector2(width/2 + 50, right * 100 + height/2), 25)

def getFrame():
    buffer = videoSub.recv()

    img_array = np.frombuffer(buffer, dtype=np.uint8)
    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    frame = np.rot90(frame)
    frame = np.flip(frame, 0)
    return frame
            
def frame2pix(frame):
    frame = pygame.surfarray.make_surface(frame)
    screen.blit(frame, (0,0))


running = True
# leftOld = joystick.get_axis(1)
# rightOld = joystick.get_axis(3)

pygame.init()
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()

start = time.time()
latest = start


while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            break
    screen.fill((0, 0, 0))

    # if keyboard.is_pressed("q"):
    #     running = False
    #     commsReq.send(b"QUIT")
    #     break
    
    # frame2pix(getFrame())
    # UI(joystick.get_axis(1),joystick.get_axis(3))
    pygame.display.flip()
    clock.tick(60)

    dt = time.time() - latest 
    latest = time.time()
    # print(f"\rFPS: {1/dt}", end = '', flush=True)
commsReq.send(b"")
pygame.quit()