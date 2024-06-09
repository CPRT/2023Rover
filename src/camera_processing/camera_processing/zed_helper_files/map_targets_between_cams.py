class PitchYaw:
    def __init__(self, p, y):
        self.pitch = p
        self.yaw = y

    def __str__(self):
        return f"Pitch: {self.pitch}, Yaw: {self.yaw}"

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f"Point({self.x}, {self.y}"

class CameraUtil:
    def __init__(self, xRes, yRes, xFov, yFov):
        self.xRes = xRes
        self.yRes = yRes
        self.xFov = xFov
        self.yFov = yFov
            
    def pitchYawFromXY(self, point: Point) -> PitchYaw:
        aX = (point.x - self.xRes / 2.0) / (self.xRes / 2.0)
        aY = (point.y - self.yRes / 2.0) / (self.yRes / 2.0)
        
        yaw = aX * self.xFov / 2.0
        pitch = aY * self.yFov / 2.0
        
        return PitchYaw(pitch, yaw)
        
    def xyFromPitchYaw(self, pitchYaw: PitchYaw) -> Point:
        aX = pitchYaw.yaw * 2.0 / self.xFov
        aY = pitchYaw.pitch * 2.0 / self.yFov
        
        x = (aX * self.xRes / 2.0) + (self.xRes / 2.0)
        y = (aY * self.yRes / 2.0) + (self.yRes / 2.0)
        
        return Point(x, y)
