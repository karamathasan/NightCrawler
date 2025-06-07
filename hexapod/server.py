import zmq

import json
import cv2
from PIL import Image
import time

ctx = zmq.Context()
videoSock = ctx.socket(zmq.PUB)
videoSock.bind("tcp://*:5555")

cap = cv2.VideoCapture(1)

def getFrame():
    success, frame = cap.read()
    if not success:
        return None
    _, buffer = cv2.imencode(".jpg",frame)
    return buffer.tobytes()

print("Starting 0MQ Server")
while True:
    videoSock.send(getFrame())
    time.sleep(0.01)