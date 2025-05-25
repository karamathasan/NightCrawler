from leg import Leg
class RobotDrive():
    """
    Driving and kinematics control for robot
    """
    # left and right legs may need a specific class for control
    def __init__(self, leftLegs:list[Leg], rightLegs: list[Leg]):
        self.left = leftLegs
        self.right = rightLegs

        self.swing_group = [leftLegs[0], rightLegs[1], leftLegs[2]]
        self.stance_group = [rightLegs[0], leftLegs[1], rightLegs[2]]
        pass

    def stand(self):
        pass

    def sit(self):
        pass

    def gait(self):
        pass

    def turn(self):
        pass

    def strafe(self):
        pass

    def balance(self):
        pass