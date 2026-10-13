import numpy as np
import Car
import requests

IP = "http://10.42.0.58"

def TransformSensorData(dist):
    return 0.0005 * dist **2 + 0.7649 * dist + 19.258

class Esp32:
    def __init__(self, connectioninfo, parent, sensors):
        self.connectioninfo = connectioninfo
        self.parent = parent
        self.sensors = sensors
        self.x = parent.x
        self.y = parent.y
        self.sending_speed = self.SetMotorSpeed(parent.speed)
        self.sending_angle = self.SetTurningAngle(parent.wheel_dir)

    def SetMotorSpeed(self, mmPerSec):
        MIN_MOVE = 200
        MAX_MOVE = 255
        CONVERSION_FACTOR = (MAX_MOVE - MIN_MOVE) / Car.MAX_SPEED
        self.sending_speed = mmPerSec * CONVERSION_FACTOR + MIN_MOVE * np.sign(mmPerSec) if abs(mmPerSec) > 5 else 0# takes -255 to -255

    def SetTurningAngle(self, rads):
        #servo takes angle from the end, and actuated angle doesn't line up with sent angle.
        degs = np.rad2deg(rads)
        self.sending_angle = 65 + degs * 1.4 #66.3 is old num
        

    def SendRequest(self, arena):
        SIM_BIAS = 0.2
        try:
                
            request_headers = {
                "turn":f"{self.sending_angle}","speed":f"{self.sending_speed}"
            }
            #print(request_headers)
            #Send an http post request with headers containing information
            response = requests.post(IP + "/", headers=request_headers, timeout=3) #the / means the main get, top file directory
            #process the response
            content_type = response.headers.get("Response-Type")
            content = response.content.decode(encoding = "utf-8")
            
            response.close()
            #split types of distance sensors (eg. BACK1, LEFT...)qq
            type_array = content_type.split(" ")
            #split content by spaces (floats but stored as a string)
            content_array = content.split(" ")
            #make a dictionary of content and types
            distances = dict(zip(type_array, content_array))
            #print(distances)
            if len(distances) > 0:
                for sensor, distance in distances.items():
                    distance = float(distance)
                    if distance > 10:
                        self.sensors[sensor].last_sensed = TransformSensorData(distance) * (1-SIM_BIAS) + self.sensors[sensor].last_sensed * SIM_BIAS
                        #print(f"{sensor} sensor hit: {distance}")
                    else:
                        self.sensors[sensor].last_sensed = self.sensors[sensor].SenseSimDist(arena)
                    print(f"{sensor}: {self.sensors[sensor].last_sensed}")
            return
        except Exception as e:
            print("Error: ", e)
            return 0
    

