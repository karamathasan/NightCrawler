import cv2

# sends data over socket server and receives processed output
class Vision():
    def __init__(self):
        self.cam = cv2.VideoCapture(0)

    def send(self):
        pass
