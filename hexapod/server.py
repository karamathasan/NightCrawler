import zmq
import asyncio

import json
import cv2
from PIL import Image
import time

ctx = zmq.Context()
readyRep = ctx.socket(zmq.REP)
processReq = ctx.socket(zmq.REQ)
videoPub = ctx.socket(zmq.PUB)

print("Starting 0MQ Server...")
readyRep.bind("tcp://*:5556")
videoPub.bind("tcp://*:5555")

cap = cv2.VideoCapture(1)

def getFrame():
    success, frame = cap.read()
    if not success:
        return None
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
    _, buffer = cv2.imencode(".jpg",frame)
    return buffer.tobytes()

print("Waiting for client")
readyRep.recv()
print("client found!")
readyRep.send(b"READY")
time.sleep(0.6)

running = True
while running:
    videoPub.send(getFrame())
    try:
        readyRep.recv(flags=zmq.NOBLOCK)
        running = False
    except:
        pass
    time.sleep(0.2)