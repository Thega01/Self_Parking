import numpy as np
import cv2 as cv
import random as rng
import Car


START_LENGTH = 345 # mm
START_OFFSET_X = 23 #mm from back wall
START_OFFSET_Y = 103 #mm from left wall (top)
START_RECT_X_OFFSET = 225 #mm to start of coloured rects form back wall
TRACK_WIDTH = 266 # mm, excluding parks
PARK_BUFFER = 20 #space between wall and first 120mm wide park
PARK_STANDOFF = 14 #mm between bumper and wall for parks
PARK_WIDTH = 120 # mm
PARK_DEPTH = 227 # mm
PARK_COUNT = 8
OCCUPIED_PARKS = 4
BORDER_WIDTH = 20 #px
TOTAL_WIDTH = START_LENGTH * 2 + PARK_BUFFER * 2 + PARK_COUNT * PARK_WIDTH
TOTAL_HEIGHT = TRACK_WIDTH + PARK_DEPTH
RECT_WIDTH = 64 #mm
HALF_RECT_WIDTH = int(RECT_WIDTH/2)
RECT_DEPTH = 20 #px
BACKGROUND_COLOUR = (255,255,255)
PARKED_CAR_COLOUR = (0,0,0)
WALL_COLOUR = (181, 228, 255)
CORNER_SENSOR_BUFFER = 100 # mm
CAR_SENSOR_BUFFER = 10 #mm
COLOUR_RECT_BUFFER = 10 #mm

COLOUR_CODE = {     #colour codes for known x, y walls in colour coded arena
    (110,0,0): (0, None),
    (120,0,0): (START_LENGTH, None),
    (130,0,0): (TOTAL_WIDTH - START_LENGTH, None),
    (140,0,0): (TOTAL_WIDTH, None),

    (0,110,0): (None, 0),
    (0,120,0): (None, TRACK_WIDTH),
    (0,130,0): (None, PARK_DEPTH)
}

VISITED = (67,67,67)



class Arena:

    def __init__(self, colours, populate = "NONE"):
        self.colours = colours

        #use 1px = 1mm
        
        #center of car starting position
        self.start_pos = (START_OFFSET_X + Car.LENGTH//2, START_OFFSET_Y + Car.WIDTH//2)

        self.car_rects = []
        self.colour_rects = []
        self.start_rects = [
            Rect(
                (START_RECT_X_OFFSET, TRACK_WIDTH),
                (START_RECT_X_OFFSET + RECT_WIDTH // 2, TRACK_WIDTH  + RECT_DEPTH), WALL_COLOUR
            ),
            Rect(
                (START_RECT_X_OFFSET + RECT_WIDTH // 2 + 1, TRACK_WIDTH),
                (START_RECT_X_OFFSET + RECT_WIDTH, TRACK_WIDTH + RECT_DEPTH), WALL_COLOUR
            ),
            # (
            #     (TOTAL_WIDTH - (START_RECT_X_OFFSET, TRACK_WIDTH),
            #     (START_RECT_X_OFFSET + RECT_WIDTH // 2, TRACK_WIDTH  + RECT_DEPTH),
            # ),
            # (
            #     (START_RECT_X_OFFSET + RECT_WIDTH // 2 + 1, TRACK_WIDTH),
            #     (START_RECT_X_OFFSET + RECT_WIDTH, TRACK_WIDTH + RECT_DEPTH),
            # ),
        ]

        for i in range(PARK_COUNT):
            park_center_x = int(START_LENGTH + PARK_BUFFER + PARK_WIDTH * (i+0.5)) # + 0.5 for center
            park_center_y = int(TRACK_WIDTH + PARK_DEPTH - PARK_STANDOFF - Car.LENGTH//2)
            park_bottom = TOTAL_HEIGHT

            car_rect = Rect((park_center_x - Car.WIDTH//2, park_center_y - Car.LENGTH//2 + CAR_SENSOR_BUFFER), (park_center_x + Car.WIDTH//2, park_center_y + Car.LENGTH//2 - CAR_SENSOR_BUFFER), PARKED_CAR_COLOUR)
            self.car_rects.append(car_rect)

            colour_rect = Rect((park_center_x - HALF_RECT_WIDTH + COLOUR_RECT_BUFFER, park_bottom), (park_center_x + HALF_RECT_WIDTH - COLOUR_RECT_BUFFER, park_bottom + RECT_DEPTH), WALL_COLOUR)
            self.colour_rects.append(colour_rect)


        self.img = np.zeros((TOTAL_HEIGHT + BORDER_WIDTH * 2, TOTAL_WIDTH + BORDER_WIDTH * 2, 3), dtype= np.uint8)
        cv.rectangle(self.img, (0, 0), (TOTAL_WIDTH + BORDER_WIDTH * 2, TOTAL_HEIGHT + BORDER_WIDTH * 2), WALL_COLOUR, cv.FILLED)

        track = Rect((0, 0), (TOTAL_WIDTH, TRACK_WIDTH), BACKGROUND_COLOUR)
        track.Draw(self.img)
        park_area = Rect((START_LENGTH, TRACK_WIDTH), (TOTAL_WIDTH - START_LENGTH, TOTAL_HEIGHT), BACKGROUND_COLOUR)
        park_area.Draw(self.img)

        for rect in self.car_rects:
            rect.Draw(self.img)

        if populate == "SIM":
            blank_img = self.img.copy()
            keyCode = ord('r')
            while keyCode == ord('r'):
                self.img = blank_img.copy()
                self._Populate(self.colour_rects + self.start_rects, self.car_rects)
                cv.imshow("preview", self.img)
                keyCode = cv.waitKey(0)

        elif populate == "CODE":
            self._ColourCode()
    

    def _Populate(self, colourRects, carRects):
        colours_list = list(self.colours.values())

        for rect in colourRects:
            rect.colour =  rng.choice(colours_list)
            rect.Draw(self.img)
        
        empty_car_rects = rng.sample(carRects, k=OCCUPIED_PARKS)

        for rect in empty_car_rects:
            rect.colour = BACKGROUND_COLOUR
            rect.Draw(self.img)

    def _ColourCode(self):
        CODE_RECTS = [
            Rect((-RECT_DEPTH, CORNER_SENSOR_BUFFER), (0, TRACK_WIDTH - CORNER_SENSOR_BUFFER), (110,0,0)),
            Rect((START_LENGTH - RECT_DEPTH, TRACK_WIDTH + CORNER_SENSOR_BUFFER), (START_LENGTH, TOTAL_HEIGHT - CORNER_SENSOR_BUFFER), (120,0,0)),
            Rect((TOTAL_WIDTH - START_LENGTH, TRACK_WIDTH + CORNER_SENSOR_BUFFER), (TOTAL_WIDTH - START_LENGTH + RECT_DEPTH, TOTAL_HEIGHT - CORNER_SENSOR_BUFFER), (130,0,0)),
            Rect((TOTAL_WIDTH, CORNER_SENSOR_BUFFER), (TOTAL_WIDTH + RECT_DEPTH, TRACK_WIDTH - CORNER_SENSOR_BUFFER), (140,0,0)),

            Rect((CORNER_SENSOR_BUFFER, -RECT_DEPTH), (TOTAL_WIDTH - CORNER_SENSOR_BUFFER, 0), (0,110,0)),
            Rect((CORNER_SENSOR_BUFFER, TRACK_WIDTH), (START_LENGTH - CORNER_SENSOR_BUFFER, TRACK_WIDTH + RECT_DEPTH), (0,120,0)),
            Rect((TOTAL_WIDTH - START_LENGTH + CORNER_SENSOR_BUFFER, TRACK_WIDTH), (TOTAL_WIDTH - CORNER_SENSOR_BUFFER, TRACK_WIDTH + RECT_DEPTH), (0,120,0)),
            Rect((START_LENGTH + CORNER_SENSOR_BUFFER, TOTAL_HEIGHT), (TOTAL_WIDTH - START_LENGTH - CORNER_SENSOR_BUFFER, TOTAL_HEIGHT + RECT_DEPTH), (0,130,0)),
        ]
        for rect in CODE_RECTS:
            rect.Draw(self.img)


def offsetPt(pt):
    return (pt[0] + BORDER_WIDTH, pt[1] + BORDER_WIDTH)


class Rect:
    def __init__(self, pt1, pt2, colour = None):
        self.pt1 = pt1
        self.pt2 = pt2
        self.colour = colour

    @property
    def center(self):
        x = (self.pt1[0] + self.pt2[0]) / 2
        y = (self.pt1[1] + self.pt2[1]) / 2
        return (x, y)

    @property
    def filled(self):
        if self.colour is not None and not np.array_equal(self.colour, WALL_COLOUR) and not np.array_equal(self.colour, BACKGROUND_COLOUR):
            return True
        return False

    def Draw(self, img):
        if self.colour is not None:
            colour = self.colour
        else:
            colour = BACKGROUND_COLOUR
        cv.rectangle(img, offsetPt(self.pt1), offsetPt(self.pt2), colour, cv.FILLED)


class RotatedRect:
    def __init__(self, center, dimensions, angle, colour = None):
        self.center = center
        self.dimensions = dimensions
        self.angle = angle
        self.colour = colour

    def BoundsXY(self):
        points = cv.boxPoints((self.center, self.dimensions, self.angle))
        x_vals = [point[0] for point in points]
        y_vals = [point[1] for point in points]

        x_vals.sort()
        y_vals.sort()

        return (x_vals[0], y_vals[0], x_vals[-1], y_vals[-1])

    def Draw(self, img):
        if self.colour is not None:
            colour = self.colour
        else:
            colour = BACKGROUND_COLOUR
        corner_points = np.intp(cv.boxPoints((offsetPt(self.center), self.dimensions, self.angle)))
        cv.fillConvexPoly(img, corner_points, colour)