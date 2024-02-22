
# Math bits got from Rev robotics sparkmax and ported into python
# https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control

class PidController:
    def __init__(self, kInvertOutput: bool, kP: float, kI: float, kD: float, kF: float, kIZone: float, kMaxOutput: float):
        self.kInvertOutput = kInvertOutput
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kF = kF
        self.kIZone = kIZone
        self.iState = 0
        self.prevErr = 0
        self.kMaxOutput = kMaxOutput
        
        self.softLimitEnabled = False
        self.softLimitHigh = 0
        self.softLimitLow = 0

    # Max Output should be raw motor output units (voltage usually)
    def setPIDValues(self, kInvertOutput: bool, kP: float, kI: float, kD: float, kF: float, kIZone: float, kMaxOutput: float):
        self.kInvertOutput = kInvertOutput
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kF = kF
        self.kIZone = kIZone
        self.kMaxOutput = kMaxOutput

    # Soft limit is meant for position PID only
    # Will prevent movement beyond a limit when the limit is exceeded.
    def enableSoftLimit(self, enabled: bool):
        self.softLimitEnabled = enabled

    # Limits should be PID units
    def setSoftLimits(self, limitLow, limitHigh):
        self.softLimitLow = limitLow
        self.softLimitHigh = limitHigh
        self.softLimitEnabled = True

    def getSoftLimits(self):
        return self.softLimitLow, self.softLimitHigh

    # Setpoint and feedback should be PID units
    def calculate(self, setpoint:float, feedback:float, arbFF:float) -> float:
        # Ensure the setpoint is within soft limits if it's enabled
        if (self.softLimitEnabled):
            setpoint = self.clampValue(setpoint, self.softLimitLow, self.softLimitHigh)
            
        # Error
        error = setpoint - feedback

        # P term
        p = error * self.kP

        # I term
        if (abs(error) <= self.kIZone or self.kIZone == 0.0):
            self.iState = self.iState + (error * self.kI)
        else:
            self.iState = 0

        # D term    
        d = (error - self.prevErr)
        self.prevErr = error
        d *= self.kD

        # F term
        f = setpoint * self.kF

        # Sum of terms
        output = p + self.iState + d + f + arbFF

        # Clamp output to given max value
        output = self.clampValue(output, -self.kMaxOutput, +self.kMaxOutput)

        # Do soft limits if it's enabled
        if (self.softLimitEnabled):

            ## Check if a soft limit is exceeded and prevent movement towards if it is
            # If: past endstop in the positive direction and going in the positive direction
            if (feedback > self.softLimitHigh and output > 0):
                output = 0
                return output
            
            # If: past endstop in the negative direction and going in the negative direction
            if (feedback < self.softLimitLow and output < 0):
                output = 0
                return output

        # Invert the direction if needed
        if (self.kInvertOutput):
            output *= -1

        return output


    
    # def calculate(self, setpoint, feedback):
    #     return self.calculate(setpoint, feedback, 0)



    ##
    ## Helper methods
    ##

    # Clamp the voltage so it doesn't exceed a given max value
    def clampValue(self, value, minValue, maxValue):
        if (value > maxValue):
            return maxValue
        elif (value < minValue):
            return minValue

        return value



    ##
    ## Individual Setters
    ##
    def setP(self, value):
        self.kP = value

    def setI(self, value):
        self.kI = value

    def setD(self, value):
        self.kD = value

    def setIZone(self, value):
        self.kIZone = value

    def setInvertOutput(self, boolean):
        self.kInvertOutput = boolean

    def setMaxOutput(self, value):
        self.kMaxOutput = value

    def setSoftLimitLow(self, value):
        self.softLimitLow = value

    def setSoftLimitHigh(self, value):
        self.softLimitHigh = value



