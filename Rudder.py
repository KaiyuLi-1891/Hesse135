import time
import math

class Rudder:
    def __init__(self, straight=90, range_value=45, rudder_interval=0.03):
        self.currentRudderPos = 0  # Current rudder position
        self.targetRudderPos = 0  # Target rudder position
        self.straight = straight  # Straight rudder position
        self.range = range_value  # Range for rudder movement
        self.rudderInterval = rudder_interval  # Time interval for rudder movement (in seconds)
        self.lastRudderTime = time.time()  # Time tracking for rudder movement

    def fuzzyTurnTo(self, bearing, directionBearing):
        difference = bearing - directionBearing

        if difference > 0:
            if difference > 180:
                self.targetRudderPos = self.straight + math.pow((360 - difference) / 180, 0.5) * self.range
            else:
                self.targetRudderPos = self.straight - math.pow(difference / 180, 0.5) * self.range
        elif difference < 0:
            if difference < -180:
                self.targetRudderPos = self.straight - math.pow((360 + difference) / 180, 0.5) * self.range
            else:
                self.targetRudderPos = self.straight + math.pow(-difference / 180, 0.5) * self.range
        else:
            self.targetRudderPos = self.straight

    def moveRudderTo(self):
        current_time = time.time()
        if current_time - self.lastRudderTime > self.rudderInterval:
            if abs(self.targetRudderPos - self.currentRudderPos) > 1:
                if self.currentRudderPos < self.targetRudderPos:
                    self.currentRudderPos += 1
                else:
                    self.currentRudderPos -= 1
                print(f'Rudder moving to position: {self.currentRudderPos}')
            self.lastRudderTime = current_time

    def rudderControl(self,x,y,psai,targetX,targetY):
        ################# The actual angular velocity of the rudder has not been considered yet ##################
        bearing = psai
        directionBearing = math.degrees(math.atan2(targetY-y,targetX-x))
        self.fuzzyTurnTo(bearing,directionBearing)
        self.moveRudderTo()
        return self.currentRudderPos
