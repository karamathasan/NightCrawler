from microdot import Microdot
from microdot.websocket import with_websocket
from microdot.cors import CORS

import ujson
import asyncio 
import network
import env
import time

from leg import Leg
from gaitDrive import GaitDrive

left1 = Leg()
left1.setJ1(90) 

print("board start")
# LED = Pin("LED",Pin.OUT)
# LED.toggle()



# motor 1 is LF, motor 2 is LB
# motor 3 is RB, motor 4 is LF

# Connect to Wi-Fi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(env.SSID, env.PASSWORD)
time.sleep(5)

# Wait for connection
max_attempts = 10
attempt = 0
print("attempting connection . . . ")
while not wlan.isconnected() and attempt < max_attempts:
    print(f"Trying to connect to env.SSID (Attempt {attempt + 1}/{max_attempts})...")
    time.sleep(10)
    attempt += 1

if wlan.isconnected():
    print("Connected to IP:", wlan.ifconfig()[0])
else:
    print("Failed to connect to Wi-Fi. Please check your SSID and password.")


app = Microdot()
CORS(app, allowed_origins = '*', allow_credentials = True)

def angle2duty(angle):
    K = (7864 - 1802) / (180)
    return int(angle * K) + 1802
    

@app.route('/hello_world', methods=['GET'])
async def get_invoices(request):
    return 'Hello World!'

@app.get('/')
@with_websocket
async def index(request, ws):
    try: 
        while True:
            data = await ws.receive()
            if not data:
                break
            await ws.send('hello world')
            
    except OSError as msg:
        print(msg)

# @app.get("/servo")
# @with_websocket
# async def index(request, ws):
#     while True:
#         data = await ws.receive()
#         data = ujson.loads(data)
#         val = data["servo"]
#         servo.duty_u16(angle2duty(val))

app.run(port=80)

