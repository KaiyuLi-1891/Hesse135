import numpy as np
import time
import math


class Sail:
    def __init__(self,Kp = 1, Ki = 0.01, Kd = 0.05):

        self.currentSailPos = 0
        self.targetSailPos = 0
        self.prevTargetPos = 0
        self.lastSailTime = 0
        self.switchTime = 0
        self.switchCheck = False

        # PID gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previousError = 0
        self.integral = 0

    def start(self):
        # self.sailServo.write(self.currentSailPos)
        pass
    def calculateStationPos(self, currentAngle):
        self.prevTargetPos = self.targetSailPos
        self.targetSailPos = currentAngle + 90
        while self.targetSailPos < 0:
            self.targetSailPos += 180
        while self.targetSailPos > 180:
            self.targetSailPos -= 180

        self.proportionTarget()
        self.checkFullRotation()

    def calculatePID(self, currentAngle):
        self.calculateTargetPos(currentAngle)
        error = self.targetSailPos - self.currentSailPos
        self.integral += error
        derivative = error - self.previousError
        self.previousError = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output

    def calculateTargetPos(self, currentAngle):
        self.prevTargetPos = self.targetSailPos

        if 15 < currentAngle < 90:
            self.targetSailPos = currentAngle - 15
        elif 270 < currentAngle < 345:
            self.targetSailPos = currentAngle + 15 - 180
        elif currentAngle >= 90 or currentAngle <= 270:
            if abs(currentAngle - 180) < 5:
                self.targetSailPos = 90
            else:
                self.targetSailPos = currentAngle / 2
        else:
            self.targetSailPos = 0
        self.targetSailPos -= 90
        if self.targetSailPos < 0:
            self.targetSailPos += 180
        self.proportionTarget()
        self.checkFullRotation()

    def proportionTarget(self):
        if abs(self.targetSailPos - self.currentSailPos) > 90:
            sailInterval = 30
        else:
            sailInterval = 30 + 70 * ((90 - abs(self.targetSailPos - self.currentSailPos)) / 90) ** 2

    def checkFullRotation(self):
        if abs(self.targetSailPos - self.prevTargetPos) > 140:
            if not self.switchCheck:
                self.switchCheck = True
                self.switchTime = time.time() * 1000  # Get current time in milliseconds
                self.targetSailPos = self.prevTargetPos
                return
            currentTime = time.time() * 1000  # Get current time in milliseconds
            if currentTime - self.switchTime < 3000:
                self.targetSailPos = self.prevTargetPos
            else:
                self.switchCheck = False
        else:
            self.switchCheck = False

    def moveSailTo(self, currentAngle):
        pidOutput = self.calculatePID(currentAngle)
        self.currentSailPos += pidOutput
        self.currentSailPos = max(0, min(180, self.currentSailPos))  # Constrain the servo movement
        # self.sailServo.write(self.currentSailPos)

    def sailControl(self,psi,gamma,angleS):
        currentAngle = 360 - gamma + psi
        currentAngle = np.mod(currentAngle, 360)
        output = self.calculatePID(currentAngle)
        return output




# Setup and main loop
def setup():
    sail = Sail()
    sail.start()
    return sail


def loop(sail):
    while True:
        # Implement your main loop logic here.
        # For example, read the current angle and call sail.moveSailTo(currentAngle).
        time.sleep(0.1)  # Delay for loop stability


# Example usage
if __name__ == "__main__":
    sail = setup()
    loop(sail)
