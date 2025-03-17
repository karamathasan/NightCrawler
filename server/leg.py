from motor import Motor
import numpy as np
class Leg():
    """
    ### Leg
    A representation of a single leg of the bot. Each joint can be controlled relative to the angle that the servos are facing
    """
    def __init__(self, right, motor1:Motor, motor2:Motor, motor3:Motor):
        self.right = right

        self.motor1 = motor1
        self.motor2 = motor2
        self.motor3 = motor3

        motor1.setBounds(-45,45)
        motor2.setBounds(-90,90)
        motor3.setBounds(-90,90)

    def setJ1(self, angle):
        self.motor1.setAngle(angle)

    def setJ2(self, angle):
        self.motor2.setAngle(angle)
    
    def setJ3(self, angle):
        self.motor3.setAngle(angle)

    def solve(self, position):
        pass