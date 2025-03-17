from leg import Leg
class GaitDrive():
    """
    Driving and kinematics control for robot
    """
    # left and right legs may need a specific class for control
    def __init__(self, leftLegs:list[Leg], rightLegs: list[Leg]):
        self.left = leftLegs
        self.right = rightLegs

        pass

    def gait():
        pass

    def turn():
        pass

    def strafe():
        pass