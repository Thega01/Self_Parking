from turtle import mode

from Sensor import PID, Camera
import Esp32
import numpy as np
import cv2 as cv
import Arena
import time as time
import RunManually


WIDTH = 80 #mm
LENGTH = 179 #mm
CAR_SIZE = (LENGTH, WIDTH)
WHEEL_SIZE = (23, 15)
TURNING_RADIUS = 245 # mm
REAL_TURNING_RADIUS = TURNING_RADIUS * 1.22
COLOUR = (80,80,80)
WHEEL_COLOUR = (0,0,0)

#distances are from back right corner. 0, 0 (car coordinates) is at the back right corner, x extends positive to the front of the car, y extends positive to the right of the car.
REAR_AXLE_POS = 20 #mm
WHEEL_INSET = 20 #mm
STEERING_AXLE_OFFSET = 8 #mm
STEERING_SPEED = 2 #rad/sec
MAX_SPEED = 33.3#26.0 #mm/sec
ACCELERATION_LIMIT = 600 #mm/sec^2
DRAG = 0.96
BACK_SENSOR_OFFSET = 12.5 #mm, distance from centerline of car to back sensors
SIDE_SENSOR_OFFSET = 16.5 #mm, distance from centerline of car to side sensors

DIRECTIONS = {
"FRONT": 0,
"LEFT": np.pi/2,
"BACK": np.pi,
"RIGHT": 3 * np.pi/2
}

def Dot(x1, y1, x2, y2):
    return x1 * x2 + y1 * y2 

def Cross(x1, y1, x2, y2):
    return x1 * y2 - x2 * y1

def DistAB(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return np.sqrt(dx**2 + dy**2)

def Normalise(angle):
    """Normalize angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def AngleAlignment(theta1, theta2): # 1 if aligned, 0 if perpendicular, -1 if facing reverse
    return Dot(np.cos(theta1), -np.sin(theta1), np.cos(theta2), -np.sin(theta2)) 

def FindIntersection(x1, y1, theta1, x2, y2, theta2):
    # Set up coefficients (Ax + By = C) for Line 1
    A1 = np.sin(theta1)
    B1 = -np.cos(theta1)
    C1 = x1 * np.sin(theta1) - y1 * np.cos(theta1)
    
    # Set up coefficients for Line 2
    A2 = np.sin(theta2)
    B2 = -np.cos(theta2)
    C2 = x2 * np.sin(theta2) - y2 * np.cos(theta2)
    
    # Construct matrices A and b
    A = np.array([
        [A1, B1],
        [A2, B2]
    ])
    b = np.array([C1, C2])
    
    try:
        # Solve the linear system
        intersection = np.linalg.solve(A, b)
        return intersection
    except np.linalg.LinAlgError:
        return (x1, y1) #lines are parallel, return the first point as a fallback


class Car:
     
    def __init__(self, centerX, centerY, direction):
        self.x = centerX
        self.y = centerY
        self.dir = direction
        self.speed = 0
        self.wheel_dir = 0
        self.wheelbase = 120 
        self.state = "UNINITIALISED"
        self.movement_state = "TRACKING_LINE"
        self.current_time = time.perf_counter() 
        self.last_time = self.current_time - 0.001
        self.distance_data = {}
        self.colour_data = {}
        self.hit_data = {}
        self.visited_colours = []

        self.sensors = { 
            "LEFT": PID(LENGTH/2 - SIDE_SENSOR_OFFSET, 0, DIRECTIONS["LEFT"], self, Arena.START_OFFSET_Y),
            "RIGHT": PID(LENGTH/2 - SIDE_SENSOR_OFFSET, WIDTH, DIRECTIONS["RIGHT"], self, Arena.TRACK_WIDTH - WIDTH - Arena.START_OFFSET_Y),
            "BACK1": PID(0, WIDTH/2 + BACK_SENSOR_OFFSET, DIRECTIONS["BACK"], self, Arena.START_OFFSET_X),
            "BACK2": PID(0, WIDTH/2 - BACK_SENSOR_OFFSET, DIRECTIONS["BACK"], self, Arena.START_OFFSET_X),
        }

        self.esp = Esp32.Esp32("connectioninfo", self, self.sensors)

        self.camera = Camera(LENGTH - 5, 28, DIRECTIONS["RIGHT"], self)



        self.camera_rays = (
            np.pi/6,
            np.pi/8,
            np.pi/12,
            0,
            -np.pi/12,
            -np.pi/8,
            -np.pi/6
        )

        self.target_x = centerX + 100
        self.target_y = centerY

        

    @property
    def x_offset(self): #offset from center X to back left corner
        return self.x + (-LENGTH * np.cos(self.dir) - WIDTH *np.sin(self.dir)) / 2

    @property
    def y_offset(self): #offset from center Y to back left corner
        return self.y + (LENGTH * np.sin(self.dir) - WIDTH *np.cos(self.dir)) / 2

    def relToAbs(self, x, y):
        absX = self.x_offset + x * np.cos(self.dir) + y * np.sin(self.dir)
        absY = self.y_offset - x * np.sin(self.dir) + y * np.cos(self.dir)
        return (absX, absY)

    @property
    def wheel_rects(self):
        return [
            (self.relToAbs(REAR_AXLE_POS, WHEEL_INSET), WHEEL_SIZE, -180/np.pi * (self.dir - self.wheel_dir)),
            (self.relToAbs(REAR_AXLE_POS, WIDTH-WHEEL_INSET), WHEEL_SIZE, -180/np.pi * (self.dir - self.wheel_dir)),
            (self.relToAbs(REAR_AXLE_POS + self.wheelbase, WHEEL_INSET), WHEEL_SIZE, -180/np.pi * self.dir),
            (self.relToAbs(REAR_AXLE_POS + self.wheelbase, WIDTH-WHEEL_INSET), WHEEL_SIZE, -180/np.pi * self.dir),
        ]

    @property
    def max_wheel_dir(self):
        return np.atan(self.wheelbase / TURNING_RADIUS)

    @property
    def front_axle_center(self):
        return (REAR_AXLE_POS + self.wheelbase, WIDTH/2)

    @property
    def dt(self):
        return (self.current_time - self.last_time) 

    def UpdateClockDiff(self):
        self.last_time = self.current_time
        self.current_time = time.perf_counter()
        #print(f"dt: {self.dt}"")

    def PerSecondToPerCycle(self, num_per_sec):
        return num_per_sec * self.dt

    def PerCycleToPerSecond(self, num_per_cycle):
        return num_per_cycle / self.dt

    def Draw(self, img):
        car_rect = Arena.RotatedRect((self.x, self.y), (LENGTH, WIDTH), -180/np.pi * self.dir, COLOUR)
        car_rect.Draw(img)
        for wheel_rect_data in self.wheel_rects:
            wheel_rect = Arena.RotatedRect(wheel_rect_data[0], wheel_rect_data[1], wheel_rect_data[2], WHEEL_COLOUR)
            wheel_rect.Draw(img)

    def DrawRays(self, active_img ,img):
        for angle in self.camera_rays:
            ray_start = (int(self.camera.x), int(self.camera.y))
            ray_end = self.camera.FindRayIntercept(img, angle)
            cv.line(img, Arena.offsetPt(ray_start), Arena.offsetPt(ray_end), (0, 0, 0), 2, cv.LINE_AA, 0)
        for sensor in self.sensors.values():
            ray_start = (int(sensor.x), int(sensor.y))
            ray_end = sensor.FindRayIntercept(active_img, 0)
            cv.line(img, Arena.offsetPt(ray_start), Arena.offsetPt(ray_end), (0, 0, 0), 2, cv.LINE_AA, 0)

    def Drive(self, target_speed):
        inst_accel_limit = self.PerSecondToPerCycle(self.PerSecondToPerCycle(ACCELERATION_LIMIT))
        inst_speed = self.PerSecondToPerCycle(self.speed)
        inst_target = self.PerSecondToPerCycle(target_speed)
        #print(f"inst_speed: {inst_speed}, inst_accel_limit: {inst_accel_limit}, target: {inst_target}")

        if inst_target > inst_speed:
            inst_speed = inst_speed + inst_accel_limit
            if inst_target < inst_speed:
                inst_speed = inst_target
        elif inst_target < inst_speed:
            inst_speed = inst_speed - inst_accel_limit
            if inst_target > inst_speed:
                inst_speed = inst_target

        self.speed = self.PerCycleToPerSecond(inst_speed)

        if self.speed > MAX_SPEED:
            self.speed = MAX_SPEED
        elif self.speed < -MAX_SPEED:
            self.speed = -MAX_SPEED

    def Turn(self, angle):
        old_angle = self.wheel_dir
        inst_steering_speed = self.PerSecondToPerCycle(STEERING_SPEED)
        dtheta = angle - self.wheel_dir
        if abs(dtheta) > inst_steering_speed:
            if dtheta > 0:
                dtheta = inst_steering_speed
            else:
                dtheta = -inst_steering_speed

        if self.wheel_dir + dtheta > self.max_wheel_dir:
            self.wheel_dir = self.max_wheel_dir
        elif self.wheel_dir + dtheta < -self.max_wheel_dir:
            self.wheel_dir = -self.max_wheel_dir
        else:
            self.wheel_dir += dtheta

        if old_angle != self.wheel_dir:
            delta = self.wheel_dir - old_angle
            self.wheelbase = 120 + STEERING_AXLE_OFFSET * np.cos(self.wheel_dir) - STEERING_AXLE_OFFSET
            rel_x_wheel_movement = STEERING_AXLE_OFFSET * np.cos(delta) - STEERING_AXLE_OFFSET
            rel_y_wheel_movement = STEERING_AXLE_OFFSET * np.sin(delta)

            interpolation_constant = ((REAR_AXLE_POS + self.wheelbase - LENGTH/2)/self.wheelbase)
            rel_x_center_movement = rel_x_wheel_movement * interpolation_constant
            rel_y_center_movement = rel_y_wheel_movement * interpolation_constant
            abs_x_center_movement = -(rel_x_center_movement * np.cos(self.dir) - rel_y_center_movement * np.sin(self.dir))
            abs_y_center_movement = -(rel_x_center_movement * np.sin(self.dir) + rel_y_center_movement * np.cos(self.dir))

            self.x = self.x - abs_x_center_movement
            self.y = self.y - abs_y_center_movement
            self.dir = self.dir + np.arctan2(rel_y_wheel_movement, self.wheelbase)

    def Move(self, real, arena):
        SLOWER_WHEN_TURNING = 0.45
        SLOWER_WHEN_REVERSING = 0.93
        inst_speed = self.PerSecondToPerCycle(self.speed) if self.speed >= 0 else self.PerSecondToPerCycle(self.speed) * SLOWER_WHEN_REVERSING
        inst_speed = inst_speed * np.cos(self.wheel_dir)**2 # Slower when turning
        self.x = self.x + inst_speed * np.cos(self.dir - self.wheel_dir/2)
        self.y = self.y - inst_speed * np.sin(self.dir - self.wheel_dir/2)
        self.dir = self.dir + inst_speed/self.wheelbase * np.tan(self.wheel_dir)

        if real:
            self.esp.SetTurningAngle(self.wheel_dir)
            self.esp.SetMotorSpeed(self.speed)
            self.esp.SendRequest(arena)

    def DistTo(self, x, y):
        """Returns the distance from the center of the car to the point"""
        return DistAB(self.x, self.y, x, y)


    def HasOvershot(self, target_x, target_y, target_dir):
        """Checks if the car has passed the target point using a dot product check."""
        dx = target_x - self.x
        dy = target_y - self.y
        dist_to_target = DistAB(self.x, self.y, target_x, target_y)
        
        if dist_to_target == 0:
            return False
            
        # Dot product: if the vector to the target opposes the target forward vector, it has overshot
        return Dot(dx / dist_to_target, dy / dist_to_target, np.cos(target_dir), -np.sin(target_dir)) < 0

    def TrackLine(self, dist_to_target, dtheta_parr, d_perp, TARGET_WIDTH):

        heading_gain = 4
        crosstrack_gain = self.max_wheel_dir / (TARGET_WIDTH)

        # Fade out cross-track pull near the target to force perfect parallel alignment
        near_target_fade = min(1.0, dist_to_target / (TARGET_WIDTH / 6))
        if self.movement_state == "TRACKING_LINE":
            return (heading_gain * dtheta_parr) + (d_perp * crosstrack_gain * near_target_fade)

        elif self.movement_state == "BACKING_UP":
            return -(heading_gain * dtheta_parr) + (d_perp * crosstrack_gain * near_target_fade)


    def MoveTo(self, target_x, target_y, target_dir):
        """Sets Car movement state, target speed and target turning angle for Rear-Wheel Steering (RWS)"""

         # ### ---PLAN--- ###
        # given current location, direction, and target location and direction, drive forwards or backwards until
        # the car can steer to land directly on the target line, then drive forwards to the target. If the car overshoots the target, back up and try again.
        #
        # practically:
        # we need to find how far (normal to the target line) the car needs to start turning.
        # if we're outside of that, starighten the car to 90 degrees to the target line and drive
        # fowards or backwards until we are within a distance of the turning point.
        # if the target is to the right of the car, and we're driving forwards, we need to turn right.
        # if the target is to the left of the car, and we're driving forwards, we need to turn left.
        # if the target is to the right of the car, and we're driving backwards, we need to turn left so that the front of the car is facing the target when we finish the turn.
        # if the target is to the left of the car, and we're driving backwards, we need to turn right.
        # we need to stop turning when the car's direction has aligned or overshot the target direction, and then track the line to the target.
        # If we overshoot the target, or the turning point isn't close to the car anymore, we need to back up and try again.
        #
        # things we need to know:
        # the distance from the car to the target line (perpendicular)
        # the distance from the car to the target point (in line with target direction)
        # the angle from the car to the target direction
        # the angle from the car to perpendicular to the target line (either way)
        # how far from the target line we need to start turning (turning radius) * (1 - cos(angle from car to target direction))
        # if the car is facing the target line
        # we could write a function to set turning angle (but not speed) given a target point and direction for line following (forwards or reverse)
        # we should write a function to find out if we've overshot the target or not
        # we should make a state machine so we know whether we're DRIVING_TOWARDS a line, TURNING_FORWARDS to a line, TURNING_BACKWARDS to a line, TRACKING_LINE a line, or BACKING_UP along a line 

        # --- CONSTANTS & THRESHOLDS ---
        CLOSE_DIST = 20        # mm
        TARGET_WIDTH = 20      # mm 
        BACKUP_DIST = REAL_TURNING_RADIUS
        LOCK_IN_ANGLE = np.pi / 60  # ~3 degrees

        # --- RELATIVE GEOMETRY ---
        d_x = target_x - self.x
        d_y = target_y - self.y
        dist_to_target = DistAB(self.x, self.y, target_x, target_y)

        # perpendicular Distance to target line (+ means target line is to the RIGHT of car heading)
        d_perp = Dot(d_x, d_y, np.sin(target_dir), -np.cos(target_dir))
        # parralel Distance along target line to point
        d_parr = np.sqrt(dist_to_target**2 - d_perp**2)
        
        # Heading error CCW [-pi, pi]
        dtheta_parr = Normalise(target_dir - self.dir)
        
        turning_dist_forwards = REAL_TURNING_RADIUS * (1 - np.cos(dtheta_parr))
        turning_dist_backwards = TURNING_RADIUS * (1 - np.cos(dtheta_parr))

        # Check if car's nose points toward the target line
        side = 1.0 if d_perp >= 0 else -1.0
        perp_angle = Normalise(target_dir - side * (np.pi / 2))
        dtheta_perp = Normalise(self.dir - perp_angle)
        is_facing_line = dtheta_parr * side >= 0
        print(f"self_dir: {self.dir:.3f}, target_dir: {target_dir:.3f}")
        print(f"sign_dist: {side}, perp_angle: {perp_angle}, dtheta_perp: {dtheta_perp:.3f}, dtheta_parr: {dtheta_parr:.3f} facing line: {is_facing_line}")

        # --- STATE MACHINE TRANSITIONS ---
        is_overshot = self.HasOvershot(target_x, target_y, target_dir)
        is_behind = self.HasOvershot(target_x, target_y, self.dir)
        dist_to_turning_zone = abs(d_perp) - turning_dist_forwards if is_facing_line else abs(d_perp) - turning_dist_backwards #> 0 if outisde, <0 if inside
        inside_turning_area = dist_to_turning_zone <= 10
        at_turning_zone = abs(dist_to_turning_zone) <= 10 
        near_turning_zone = abs(dist_to_turning_zone) <= 20

        print(f"d_perp: {d_perp:.0f}, is_facing_line: {is_facing_line}, inside_turning_area: {inside_turning_area}, dtheta_perp: {dtheta_perp:.2f}, dtheta_parr: {dtheta_parr:.2f}")

        if is_overshot:
            if not inside_turning_area:
                self.movement_state = "DRIVING_TOWARDS"
            else: self.movement_state = "BACKING_UP"
        elif is_behind and self.movement_state != "TURNING_BACKWARDS" and not inside_turning_area:
            self.movement_state = "DRIVING_TOWARDS"
        elif self.movement_state == "BACKING_UP":
            if dist_to_target > BACKUP_DIST:
                self.movement_state = "DRIVING_TOWARDS"
        

        elif self.movement_state == "TURNING_FORWARDS":
            if is_facing_line:
                if near_turning_zone:
                    if abs(d_perp) <= TARGET_WIDTH / 2:
                        self.movement_state = "TRACKING_LINE"
                else:
                    if inside_turning_area: self.movement_state = "TRACKING_LINE"
                    else: self.movement_state = "DRIVING_TOWARDS"
            else: self.movement_state = "DRIVING_TOWARDS"

        elif self.movement_state == "TURNING_BACKWARDS":
            if not is_facing_line:
                if near_turning_zone:
                    if is_facing_line and abs(d_perp) <= TARGET_WIDTH / 2:
                        self.movement_state = "TRACKING_LINE"
                else:
                    if inside_turning_area: self.movement_state = "TRACKING_LINE"
                    else: self.movement_state = "DRIVING_TOWARDS" 
            else: self.movement_state = "DRIVING_TOWARDS" 

        elif self.movement_state == "TRACKING_LINE":
            if not (inside_turning_area or near_turning_zone):
                self.movement_state = "DRIVING_TOWARDS"
            elif not abs(d_perp) <= TARGET_WIDTH / 2:
                self.movement_state = "TURNING_FORWARDS" if is_facing_line else "TURNING_BACKWARDS"

        elif self.movement_state == "DRIVING_TOWARDS":
            if at_turning_zone:
                if is_facing_line:
                    self.movement_state = "TURNING_FORWARDS"
                else:
                    self.movement_state = "TURNING_BACKWARDS"
            elif (inside_turning_area or near_turning_zone) and abs(dtheta_parr) < LOCK_IN_ANGLE:
                self.movement_state = "TRACKING_LINE"

        # --- STATE EXECUTIONS ---
        target_speed = 0.0
        target_wheel_dir = 0.0

        if self.movement_state == "DRIVING_TOWARDS":
            if is_facing_line:
                target_speed = MAX_SPEED
                target_wheel_dir = -dtheta_perp
            else:
                target_speed = - MAX_SPEED
                target_wheel_dir = dtheta_perp
            if inside_turning_area:
                target_speed = -target_speed
                target_wheel_dir = -target_wheel_dir

        elif self.movement_state == "TURNING_FORWARDS":
            target_speed = MAX_SPEED
            target_wheel_dir = np.sign(d_perp) * self.max_wheel_dir

        elif self.movement_state == "TURNING_BACKWARDS":
            target_speed = -MAX_SPEED
            target_wheel_dir = np.sign(d_perp) * self.max_wheel_dir

        elif self.movement_state == "TRACKING_LINE":
            target_speed = min(MAX_SPEED, 2 * ACCELERATION_LIMIT * dist_to_target)
            target_wheel_dir = self.TrackLine(dist_to_target, dtheta_parr, d_perp, TARGET_WIDTH)
            

        elif self.movement_state == "BACKING_UP":
            target_speed = -MAX_SPEED
            target_wheel_dir = self.TrackLine(dist_to_target, dtheta_parr, d_perp, TARGET_WIDTH)

        # Clamp steering input to max physical limit
        target_wheel_dir = np.clip(target_wheel_dir, -self.max_wheel_dir, self.max_wheel_dir)

        # Move
        self.Turn(target_wheel_dir)
        self.Drive(target_speed)

        




    def DoSensorsAndStates(self, generated_arena, code_arena, arena, ignored_colours, real, auto):
        WAYPOINT_TOLERANCE = 10 # mm
        for sensor in self.esp.sensors.values():
            if real:
                self.distance_data[sensor] = sensor.SenseRealDist()
            else:
                self.distance_data[sensor] = sensor.SenseSimDist(generated_arena)
            self.hit_data[sensor] = sensor.FindHitData(code_arena)
            #print(f"{sensor} hit at {sensor.FindHitData(code_arena)}")

        if real:
            self.camera.UpdateImage()
            self.LocateOnTrack()
        for angle in self.camera_rays:
            if real:
                self.colour_data[angle] = self.camera.SenseRealColour(angle)
            else:
                self.colour_data[angle] = self.camera.SenseSimColour(generated_arena, angle)
            if self.colour_data[angle] is not None:
                #print(f"colour {self.colour_data[angle]} at angle {angle}")
                if not self.state == "PARKING":
                    self.InterpretCameraData(arena, code_arena, angle, self.colour_data[angle], ignored_colours)
        
        if not auto:
             RunManually.MoveManually(self)
             return
         
        if self.state == "FINDING_START_COLOURS":
            self.MoveTo(self.target_x, self.target_y, 0)
               
        elif self.state == "SEARCHING_FOR_TARGET":
            #print(self.DistTo(self.target_x,self.target_y))
            self.FindTarget(arena)
            self.MoveTo(self.target_x, self.target_y, 0)
            if self.DistTo(self.target_x, self.target_y) < WAYPOINT_TOLERANCE and self.state == "SEARCHING_FOR_TARGET":
                self.state = "DONE"
                
        elif self.state == "PREPARING_TO_PARK":
            self.FindTarget(arena)
            self.MoveTo(self.target_x, self.target_y, 0)
            if self.DistTo(self.target_x, self.target_y) < WAYPOINT_TOLERANCE:
                self.state = "PARKING"
                self.speed = 1

        elif self.state == "PARKING":
            self.FindTarget(arena)
            self.MoveTo(self.target_x, self.target_y, np.pi* 3 / 2)
            if self.DistTo(self.target_x, self.target_y) < WAYPOINT_TOLERANCE and self.state == "PARKING": 
                self.state = "PARKED"

        elif self.state == "PARKED":
            self.speed = 0
            self.Move(real, arena)
            
            cv.waitKey(0)
            self.UpdateClockDiff()
            self.FindTarget(arena)
            if self.state != "REVERSING_OUT":
                self.state = "DONE"
            else:
                self.speed = -1

        elif self.state == "REVERSING_OUT":
            self.MoveTo(self.target_x, self.target_y, 0)
            if self.DistTo(self.target_x, self.target_y) < WAYPOINT_TOLERANCE:
                self.state = "SEARCHING_FOR_TARGET"
                self.speed = 1



        print(self.state, self.movement_state)
        #print(f"Target: {self.target_x}, {self.target_y}")
        #print(f"Dist to: {self.DistTo(self.target_x, self.target_y)}")
        #print(f"location: {self.x}, {self.y}")

    def LocateOnTrack(self):
        SIM_BIAS = 0.5
        ANGLE_IGNORED = np.pi/6
        inst_speed = self.PerSecondToPerCycle(self.speed)
        x_guess = self.x
        y_guess = self.y
        dir_guess = self.dir + inst_speed/self.wheelbase * np.tan(self.wheel_dir)

        x_guesses = {}
        y_guesses = {}

        hit_1d = {}

        for sensor in self.esp.sensors.values():
            hit = self.hit_data[sensor]
            dist = self.distance_data[sensor]

            if hit[0] is not None:
                x_guesses[sensor] = hit[0] + (self.x - sensor.x) - dist * np.cos(sensor.dir)
                hit_1d[sensor] = hit[0]

            if hit[1] is not None:
                y_guesses[sensor] = hit[1] + (self.y - sensor.y) + dist * np.sin(sensor.dir)
                hit_1d[sensor] = hit[1]
        
        x_guesses = {sensor: val for sensor, val in x_guesses.items() if np.isfinite(val)}
        y_guesses = {sensor: val for sensor, val in y_guesses.items() if np.isfinite(val)}

        if len(x_guesses) > 0:
            x_guess = np.median(list(x_guesses.values()))

        if len(y_guesses) > 0:
            y_guess = np.median(list(y_guesses.values()))

        dir_guesses = []
        #only make guessses on direction if we are not in line with x or y axis
        quartile_dir = (self.dir + np.pi) % (np.pi / 2)
        if (quartile_dir > ANGLE_IGNORED and quartile_dir < np.pi/2 - ANGLE_IGNORED):
        
            DistBetweenSensors = lambda sensor1, sensor2: np.sqrt((sensor1.x - sensor2.x)**2 + (sensor1.y - sensor2.y)**2)
            DistBetweenGuesses = lambda sensor1, sensor2, guessesDict: guessesDict[sensor2] - guessesDict[sensor1]
            PrettyClose = lambda num1, num2: abs(num1 - num2) < 5

            

            for guess_dict in (x_guesses, y_guesses):
                if len(guess_dict) >= 2:
                    sensors = list(guess_dict.keys())
                    for i, sensor1 in enumerate(sensors):
                        for sensor2 in sensors[i+1:]:
                            if PrettyClose(hit_1d[sensor1], hit_1d[sensor2]):
                                if sensor1.dir_offset == sensor2.dir_offset:
                                    pass
                                    # #super inaccurate at any distance due to small distance between sensors.
                                    # dir_guesses.append(
                                    #     np.atan2(
                                    #         DistBetweenGuesses(sensor1, sensor2, self.distance_data),
                                    #         DistBetweenSensors(sensor1, sensor2)
                                    # ))
                                    # print(f"dir_guess from two back sensors: {dir_guesses[-1]}")
                                else:
                                    dir_guesses.append(
                                        np.atan2(
                                            self.distance_data[sensor1] 
                                            + sensor1.x_offset * np.cos(sensor1.dir_offset) 
                                            + sensor1.y_offset * np.sin(sensor1.dir_offset)
                                            ,
                                            self.distance_data[sensor2] 
                                            + sensor2.x_offset * np.sin(sensor2.dir_offset)
                                            + sensor2.y_offset * np.cos(sensor2.dir_offset)
                                    ))
                                    print(f"dir_guess from a side and a back sensor: {dir_guesses[-1]}")
                            else:
                                #pass #also super innacurate, to the point I think my maths is wrong
                                dir_guesses.append(
                                    np.sign(self.dir) * np.arccos(
                                        np.clip(
                                            DistBetweenGuesses(sensor1, sensor2, hit_1d) /
                                            (self.distance_data[sensor1] + self.distance_data[sensor2] + DistBetweenSensors(sensor1, sensor2)),
                                            -1, 1
                                        )
                                ))
                                print(f"dir_guess from two side sensors: {dir_guesses[-1]}")
            #print(f"simulated dir: {self.dir}")
            NonNaNSubset = lambda arr: [item for item in arr if np.isfinite(item)]
            
            dir_guesses = NonNaNSubset(dir_guesses)

            if len(dir_guesses) > 0:
                #print(f"dir_guesses: {dir_guesses}")
                dir_guess = np.median(dir_guesses)

        newCarRect = Arena.RotatedRect((x_guess, y_guess), (LENGTH, WIDTH), -180/np.pi * dir_guess, COLOUR)
        (min_x, min_y, max_x, max_y) = newCarRect.BoundsXY()

        in_bounds = min_x >0 and min_y > 0 and max_x < Arena.TOTAL_WIDTH and max_y < Arena.TOTAL_HEIGHT
        
        if in_bounds:
            dist_from_last = self.DistTo(x_guess, y_guess)
            print(f"Updating to x:{self.x}, y:{self.y}, dir:{dir_guess}")
            if dist_from_last < 50:
                self.x = self.x * SIM_BIAS + x_guess * (1-SIM_BIAS)

            if dist_from_last < 50:
                self.y = self.y * SIM_BIAS + y_guess * (1-SIM_BIAS)

            if abs(Normalise(dir_guess - self.dir)) < np.pi/6:
                self.dir = self.dir * SIM_BIAS + dir_guess * (1-SIM_BIAS)

    def PointInRect(self, x, y, rect):
        if rect.pt1[0] <= x and rect.pt2[0] >= x and rect.pt1[1] <= y and rect.pt2[1] >= y:
            return True
        else:
            return False

    def InterpretCameraData(self, arena, code_arena, angle, colour, ignored_colours):
        colour = tuple(int(c) for c in colour)
        #print(f"colour observed: {colour} at angle {angle}")
        if colour != Arena.PARKED_CAR_COLOUR:
            while np.array_equal(self.camera.SenseSimColour(arena, angle), Arena.PARKED_CAR_COLOUR): 
                (x, y) = self.camera.FindRayIntercept(arena.img, angle)
                full_car_rects =  [rect for rect in arena.car_rects if rect.colour is not None]
                for car_rect in full_car_rects:
                    if self.PointInRect(x ,y, car_rect):
                        car_rect.colour = None  
                        car_rect.Draw(arena.img)
                        car_rect.Draw(code_arena.img)
        
        (x, y) = self.camera.FindRayIntercept(arena.img, angle)
        if not any(np.array_equal(colour, bad_colour) for bad_colour in ignored_colours):
            #print(f"drawing colour: {colour}")
            for rect in (arena.start_rects):
                if self.PointInRect(x, y, rect):
                    rect.colour = colour
                    rect.Draw(arena.img)
                    if self.state == "FINDING_START_COLOURS":
                        self.state = "SEARCHING_FOR_TARGET"
                        self.target_x = Arena.TOTAL_WIDTH - arena.start_pos[0]
                        self.target_y = arena.start_pos[1]

        valid_park_colours = []
        for start_rect in arena.start_rects:
            if start_rect.filled:
                valid_park_colours.append(start_rect.colour)

        for valid_colour in valid_park_colours:
            if np.array_equal(colour, valid_colour):
                for rect in arena.colour_rects:
                    if not rect.filled and self.PointInRect(x, y, rect):
                        rect.colour = colour
                        rect.Draw(arena.img)

    def FindTarget(self, arena):
        if self.state == "DONE" or self.state == "REVERSING OUT":
            pass

        empty_parks = [park for park in arena.car_rects if not park.filled]
        target_rects = [rect for rect in arena.colour_rects if rect.filled and not any(np.array_equal(colour, rect.colour) for colour in self.visited_colours)] # very long line # very useful comment
        target_parks = []
        target_park_rects = []
        for park in empty_parks:
            for rect in target_rects:
                if park.center[0] == rect.center[0]:
                    target_parks.append(park)
                    target_park_rects.append(rect)


        if len(target_parks) > 0:
            if self.state == "SEARCHING_FOR_TARGET":
                self.state = "PREPARING_TO_PARK"
            target_park = sorted(target_parks, key = lambda rect: rect.pt1[0])[0]
            target_park_rect = sorted(target_park_rects, key = lambda rect: rect.pt1[0])[0]
            if self.state == "PARKED":
                self.visited_colours.append(target_park_rect.colour)
                self.state = "REVERSING_OUT"
                self.target_x = target_park.center[0] + 50# - REAL_TURNING_RADIUS
                self.target_y = arena.start_pos[1]
            if self.state == "PARKING":
                (self.target_x, self.target_y) = target_park.center
                return
            if self.state == "PREPARING_TO_PARK":
                self.target_x = target_park.center[0] - REAL_TURNING_RADIUS
                self.target_y = arena.start_pos[1]
                return
        else:
            
            self.state = "SEARCHING_FOR_TARGET"
            self.target_x = min(Arena.TOTAL_WIDTH - arena.start_pos[0], self.x + LENGTH * 1.5)
            self.target_y = arena.start_pos[1]
        