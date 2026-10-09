import random as rng
import Car
import Arena
import numpy as np
import cv2 as cv
from pydualsense import pydualsense
import RunManually
import CameraConnection

COLOURS = { #BGR
    "RED": (0, 0, 255),
    "GREEN": (0, 255, 0),
    "BLUE": (255, 0, 0),
    "YELLOW": (0, 255, 255)
}

IGNORED_COLOURS = {
    "BACKGROUND": Arena.BACKGROUND_COLOUR,
    "WALL": Arena.WALL_COLOUR,
    "CAR": Arena.PARKED_CAR_COLOUR
}

def FindControllerXY(ds):
    x = ds.state.RX
    y = ds.state.LY

    if abs(x) < 10:
        x = 0
    if abs(y) < 10:
        y = 0


    pct_x = (x + 0.5) / 127.5
    pct_y = -(y + 0.5) / 127.5

    car_lr = -np.pi/7 * pct_x
    car_fb = Car.MAX_SPEED * pct_y
    print(f"left/right: {car_lr}, foward/back: {car_fb}")
    return (car_lr, car_fb)


# def RunManually(arena, generated_arena, car, real):
#     ds = pydualsense() # open controller
#     ds.init() # initialize controller
#     ds.light.setColorI(80,0,255) # set touchpad color to purple

#     while (True):
#         car.UpdateClockDiff()
#         displayArena = generated_arena.img.copy()
#         sensedArena = arena.img.copy()
#         (steer, throttle) = FindControllerXY(ds)
#         # if real:
#         #     RealCar.Drive(throttle)
#         #     RealCar.Turn(steer)
#         car.Turn(steer)
#         car.Drive(throttle)
#         car.Move(real, arena)
#         car.Draw(displayArena)
#         car.Draw(sensedArena)
#         cv.imshow("preview", displayArena)
#         cv.imshow("sensed", sensedArena)
#         if cv.waitKey(1) == ord('q') or ds.state.cross:
#             break

#     return



def RunCar(arena, generated_arena, code_arena, car, real, auto):
    car.state = "FINDING_START_COLOURS"

    #camera_subprocess = subprocess.Popen([sys.executable, "CameraConnection.py"]) #starts getting images from camera
    car.esp.SetMotorSpeed(0)
    car.esp.SetTurningAngle(0)
    if real:
        for i in range(10, 0, -1):
            print (i)
            CameraConnection.ConfigureCamera()
            car.camera.CameraOn(is_on = True)
            car.camera.UpdateImage()
            car.esp.SendRequest(arena)
            car.CalibrateSensors(arena)
            cv.waitKey(400)


    car.UpdateClockDiff()
    while (car.state != "DONE"):
        car.UpdateClockDiff()
        car.DoSensorsAndStates(generated_arena, code_arena, arena, IGNORED_COLOURS.values(), real, auto)
        
        displayArena = generated_arena.img.copy()
        sensedArena = arena.img.copy()
        car.Move(real, arena)
        car.Draw(displayArena) 
        car.Draw(sensedArena)
        car.DrawRays(arena.img, sensedArena)
        try:
            cv.drawMarker(sensedArena, Arena.offsetPt((int(car.target_x), int(car.target_y))), (120, 80, 200), cv.MARKER_DIAMOND, 10, 5 )
        except:
            print(car.target_x)
            print(car.target_y)
            break
        cv.imshow("simulated", displayArena)
        cv.imshow("sensed", sensedArena)
        keyCode = cv.waitKey(1)

        if keyCode == ord('q'):
            break
    return




seed = 1
real = True
auto = True
start_angle = (rng.random() * 2 - 1) * 0.05 if not real else 0
while True:
    rng.seed(seed)
    generated_arena = Arena.Arena(COLOURS, "SIM") if not real else Arena.Arena(COLOURS)
    code_arena = Arena.Arena(COLOURS, "CODE")
    arena = Arena.Arena(COLOURS)
    (x, y) = arena.start_pos
    car = Car.Car(x, y, start_angle)        
    if not auto:
        RunManually.StartController()
    #RunManually(arena, generated_arena, car, real)
    
   
    RunCar(arena, generated_arena, code_arena, car, real, auto)
    car.speed = 0
    car.Move(real, arena)
    cv.waitKey(0)
    car.esp.SendRequest()
    if cv.waitKey(0) != ord('r'):
        break
    else:
        seed = seed + 1
        
    if real:
        car.camera.CameraOn(is_on = False)