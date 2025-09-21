import asyncio
import keyboard
import time
import threading
from collections import deque

import zmq
import pygame
import cv2
import numpy as np

from transformers import pipeline
from PIL import Image


'''
control scheme:
left stick: strafe
right stick: rotate
'''
# def quitApp_thread():
#     while True:
#         if keyboard.is_pressed("q"):
#             commsReq.send(b"QUIT")
#             commsReq.disconnect("")
#             quit()
# quit_thread = threading.Thread(target=quitApp_thread,daemon=True)
# quit_thread.start()


frameq = []
depthq = []

hx_addr = "tcp://100.121.88.110:"
ctx = zmq.Context()
commsReq = ctx.socket(zmq.REQ)
depthPub = ctx.socket(zmq.PUB)
sensorSub = ctx.socket(zmq.SUB)
videoSub = ctx.socket(zmq.SUB)

depthPub.connect(hx_addr + "5557")
videoSub.connect(hx_addr + "5555")
videoSub.setsockopt(zmq.SUBSCRIBE, b'')
commsReq.connect("tcp://100.121.88.110:5556")
commsReq.setsockopt(zmq.RCVTIMEO, 5000)

commsReq.send(b"READY")
print("READY sent. awaiting messge")
ready = commsReq.recv()
print("Connected to address")

# def UI(left, right):
#     pygame.draw.line(screen,"red",pygame.Vector2(width/2 - 50,height/2), pygame.Vector2(width/2 - 50, left * 100 + height/2), 25)
#     pygame.draw.line(screen,"blue",pygame.Vector2(width/2 + 50,height/2), pygame.Vector2(width/2 + 50, right * 100 + height/2), 25)

def sendDepth_thread():
    while True:
        pipe = pipeline(task="depth-estimation", model="depth-anything/Depth-Anything-V2-Small-hf")
        if latest_frame is not None:
            image = Image.fromarray(latest_frame)
            depth = pipe(image)["depth"]
            if depth != latest_depth:
                print(type(depth))
                depthq.append(depth)
                # depthPub.send()

def getFrame_thread():
    while True:
        buffer = videoSub.recv()

        img_array = np.frombuffer(buffer, dtype=np.uint8)
        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        # frame = cv2.resize(frame, (640,480))
        frame = np.rot90(frame)
        frame = np.flip(frame, 0)
        frameq.append(frame) 

def frame2pix(frame):
    frame = pygame.surfarray.make_surface(frame)
    screen.blit(frame, (0,0))


def depth2pix(depth):
    depth = pygame.surfarray.make_surface(depth)
    screen.blit(depth, (641,0))

# joystick = pygame.joystick.Joystick(0)
# rightStick = pygame.joystick.Joystick(1)
# pygame.joystick.init()

pygame.init()
height = 920
width = 1280
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()
running = True

frame_thread = threading.Thread(target=getFrame_thread, daemon=True)
frame_thread.start()

depth_thread = threading.Thread(target=sendDepth_thread, daemon=True)
depth_thread.start()

latest_depth = None
latest_frame = None
# leftOld = joystick.get_axis(1)
# rightOld = joystick.get_axis(3)


start = time.time()
latest = start
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            break
    screen.fill((0, 0, 0))


    if len(frameq) > 0:
        latest_frame = frameq.pop(0)
    if latest_frame is not None: frame2pix(latest_frame)

    if len(depthq) > 0:
        latest_depth = depthq.pop(0)
    if latest_depth is not None: depth2pix(latest_depth)

    # UI(joystick.get_axis(1),joystick.get_axis(3))
    pygame.display.flip()
    clock.tick(60)

    dt = time.time() - latest 
    latest = time.time()
    print(f"\rFPS: {1/dt}", end = '', flush=True)
commsReq.send(b"")
pygame.quit()