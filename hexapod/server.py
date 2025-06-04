# going to use sockets instead
import socket
import socketserver

import json
from flask import Flask, Request, Response, render_template, render_template_string
from flask_cors import CORS
import cv2
from PIL import Image

cap = cv2.VideoCapture(1)

# app = Flask(__name__)
app = Flask("server.py")
CORS(app)

def generateFrame():
    while True:
        success, frame = cap.read()
        if not success:
            break
        _, buffer = cv2.imencode(".jpg",frame)
        fbytes = buffer.tobytes()
        yield(b'--frame\r\n'
              b'Content-Type: image/jpeg\r\n\n' + fbytes + b'\r\n')

@app.route("/")
def home():
    # return json.dumps({"message":"hello"})

    return Response(generateFrame(), mimetype='multipart/x-mixed-replace; boundary=frame')
    # return "<h>test</h>"




app.run(host="0.0.0.0",port=5000,debug=True)