import asyncio
import json
import time
import threading
# from collecions import deque

import zmq
import cv2
from PIL import Image

op_addr = "tcp://100.126.75.120:" #laptop ip
# op_addr = "tcp://100.125.63.1:" #pc ip
ctx = zmq.Context()
commsRep = ctx.socket(zmq.REP)
depthSub = ctx.socket(zmq.SUB)
videoPub = ctx.socket(zmq.PUB)
# videoPub.setsockopt(zmq.LINGER, 100)

print("Starting 0MQ Server...")
depthSub.connect(op_addr+"5557")
depthSub.setsockopt(zmq.SUBSCRIBE, b'')
commsRep.bind("tcp://*:5556")
videoPub.bind("tcp://*:5555")

cap = cv2.VideoCapture(0)
assert cap.read()[0] 
CAMERA_RES = (640,480)

def getFrame():
    success, frame = cap.read()
    if not success:
        return None
    dw = int(CAMERA_RES[0]//4)
    dh = int(CAMERA_RES[1]//4)

    # frame = cv2.resize(frame, (dw,dh))
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    _, buffer = cv2.imencode(".jpg",frame)
    return buffer.tobytes()

# comms_log = []
# cap = 50
# def comms():
#     b = commsRep.recv()
#     commsRep.send("ACK")
#     comms_log.insert(str(b),0)
#     if len(comms_log) > cap:
#         comms_log.pop()

bufferq = []
buffer_dt = 0.0001
def sendFrame():
    start = time.time()
    latest_buffer_time = start
    
    while True:
        global buffer_dt
        success, frame = cap.read()
        if not success:
            return None
        dw = int(CAMERA_RES[0]//4)
        dh = int(CAMERA_RES[1]//4)

        frame = cv2.resize(frame, (dw,dh))
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
        # _, buffer = cv2.imencode(".jpg",frame)
        _, buffer = cv2.imencode(".jpg",frame,encode_param)
        bufferq.append(buffer)
        if len(bufferq) > 0:
            try:
                frame_buffer = bufferq.pop()
                # videoPub.send(frame, copy=False, flags=zmq.NOBLOCK) #does not output anything
                videoPub.send(frame_buffer, copy = False)
                buffer_dt = time.time() - latest_buffer_time
                latest_buffer_time = time.time()
            except:
                pass

frame_send_thread = threading.Thread(target = sendFrame, daemon=True)
frame_send_thread.start()

print("Waiting for client")
commsRep.recv()
print("client found!")
commsRep.send(b"READY")
time.sleep(0.6)

running = True
start = time.time()
latest = start

while running:
    try:
        commsRep.recv(flags=zmq.NOBLOCK)
        running = False
    except:
        pass
        
    time.sleep(0.016)
    dt = time.time() - latest 
    latest = time.time()
    print(f"\rFPS: {1/dt:6.6f} -- Buffer FPS: {1/buffer_dt:6.6f}", end = '', flush=True)