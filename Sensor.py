import numpy as np
import Arena
import ColorCheck as ColourCheck
import Image
import subprocess
import sys
import cv2 as cv
import Car

class Sensor:
    def __init__(self, x, y, dir, parent):
        self.x_offset = x
        self.y_offset = y
        self.dir_offset = dir
        self.parent = parent
        print(f"Initial Sensor location: {x}, {y}")

    @property
    def x(self):
        return self.parent.relToAbs(self.x_offset, self.y_offset)[0]

    @property
    def y(self):
        return self.parent.relToAbs(self.x_offset, self.y_offset)[1]

    @property
    def dir(self):
        return Car.Normalise(self.dir_offset + self.parent.dir)

    
    def FindRayIntercept(self, img, directionOffset):
        startX = self.x + Arena.BORDER_WIDTH
        startY = self.y + Arena.BORDER_WIDTH
        START_DIST = Car.WIDTH
        COARSE_DIST = 10 #px
        FINE_DIST = 1 #px

        COARSE_X = COARSE_DIST * np.cos(self.dir + directionOffset)
        COARSE_Y = -COARSE_DIST * np.sin(self.dir + directionOffset)

        FINE_X = FINE_DIST * np.cos(self.dir + directionOffset)
        FINE_Y = - FINE_DIST * np.sin(self.dir + directionOffset)

        scanX = startX + START_DIST * np.cos(self.dir + directionOffset)
        scanY = startY - START_DIST * np.sin(self.dir + directionOffset)

        while (0 <= int(scanY) < img.shape[0] and 0 <= int(scanX) < img.shape[1] 
               and np.all(img[int(scanY), int(scanX)] == Arena.BACKGROUND_COLOUR)):
            scanX = scanX + COARSE_X
            scanY = scanY + COARSE_Y

        scanX = scanX - COARSE_X
        scanY = scanY - COARSE_Y

        while (0 <= int(scanY) < img.shape[0] and 0 <= int(scanX) < img.shape[1] 
               and np.all(img[int(scanY), int(scanX)] == Arena.BACKGROUND_COLOUR)):
            scanX = scanX + FINE_X
            scanY = scanY + FINE_Y
        
        finalX = max(0, min(int(scanX) - Arena.BORDER_WIDTH, Arena.TOTAL_WIDTH + 1)) 
        finalY = max(0, min(int(scanY) - Arena.BORDER_WIDTH, Arena.TOTAL_HEIGHT + 1))
        
        return (finalX, finalY)


class Camera(Sensor):
    def __init__(self, x, y, dir, parent):
            super().__init__(x, y, dir, parent)
            self.last_img = Image.ReadImage()

    def CurrentImage(self):
        return self.last_img
    
    def UpdateImage(self):
        self.last_img = Image.ReadImage()
        
    def SenseSimColour(self, arena, ray_angle):
            img = arena.img
            (x, y) = self.FindRayIntercept(img, ray_angle)
            colour = tuple(int(c) for c in img[y + Arena.BORDER_WIDTH, x + Arena.BORDER_WIDTH])
            #print(f"{colour} at {x}, {y}")
            return colour
    
    def CameraOn(self, is_on):
        if is_on:
            ColourCheck.ScanStart(Image.ReadImage())
        else:
            #camera_subprocess.kill()
            pass

    def SenseRealColour(self, ray_angle):
        colour = Image.SenseRealColour(ray_angle, self.last_img)
        #print(f"image hash: {hash(self.last_img.tobytes())}")
        return colour

class PID(Sensor):
    def __init__(self, x, y, dir, parent, initial_sense):
        super().__init__(x, y, dir, parent)
        self.last_sensed = initial_sense

    def SenseSimDist(self, arena):
        img = arena.img
        #print(f"Current Sensor location: {self.x}, {self.y}")
        (x, y) = self.FindRayIntercept(img, 0)
        dx = x - self.x
        dy = y - self.y
        dist = np.sqrt(dx**2 + dy**2)
        #print(f"sensor dists: x{dx} y{dy} dist{dist}")
        return dist

    def FindHitData(self, code_arena):
        img = code_arena.img
        (x, y) = self.FindRayIntercept(img, 0)
        colour_code = tuple(int(c) for c in img[y + Arena.BORDER_WIDTH, x + Arena.BORDER_WIDTH])
        (known_x, known_y) = Arena.COLOUR_CODE.get(colour_code, (None, None))
        return (known_x, known_y)


    def SenseRealDist(self):
        return self.last_sensed