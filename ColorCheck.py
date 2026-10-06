import numpy as np 
import cv2 as cv
import sys

#my global colours

#colour 1
real_color1 = np.array([])
lowerColor1 = np.array([])
upperColor1 = np.array([])
#colour 2
real_color2 = np.array([])
lowerColor2 = np.array([])
upperColor2 = np.array([])

range = 8 #range for hue
satRange = 60 #saturation range
valRange = 60 #range for value

BLACK_THRESH = 30 # value cutoff for black

CAMERA_VIEW_ANGLE = 67.59 #total camera view angle
START_SCAN_ANGLE = 65 - 57 #ACW from 0 degrees
BLOCK_SEPARATION = 5 #pixels between the two start colours
BLOCK_WIDTH = 8 #width of start scan colour avg blocks
SCAN_THICKNESS = 10  #vertical height of our scan band (both normal scans and start scan)
SCAN_HEIGHT = 10 #how far to offset scan from center height
SCAN_WIDTH = 3 #the width on either side of the scan angle
#begin function
#scans the two colours stores in global  colours
def ScanStart(image):
    try: 
        global range
        global satRange
        global valRange
        global lowerColor1
        global upperColor1
        global lowerColor2
        global upperColor2
        global real_color1
        global real_color2
        #get the height and width of the image
        height, width, channels = image.shape
        #get the halfway points
        scan_angle = int(START_SCAN_ANGLE*width/CAMERA_VIEW_ANGLE)
        halfheight = int(height/2)
        #print(halfheight)
        #print(scan_angle)
        
        #the two blocks are at center height, in th middle with 200 pixels beween each block
        #get the two blocks and average their h,s,v values
        block1 = image[halfheight-SCAN_THICKNESS :halfheight+SCAN_THICKNESS, scan_angle-BLOCK_WIDTH-BLOCK_SEPARATION:scan_angle-BLOCK_SEPARATION]
        block1 = cv.resize(block1, (200, 200), cv.INTER_CUBIC)
        #cv.imshow('block 1: ', block1)
        block1 = cv.cvtColor(block1, cv.COLOR_BGR2HSV)
        myHue1 = int(np.mean(block1[:,:, 0]))
        mySat1 = int(np.mean(block1[:,:, 1]))
        myVal1 = int(np.mean(block1[:,:, 2]))
        real_color1 = np.array([myHue1, mySat1, myVal1])
        
        block2 = image[halfheight-SCAN_THICKNESS:halfheight+SCAN_THICKNESS, scan_angle+BLOCK_SEPARATION:scan_angle+BLOCK_WIDTH+BLOCK_SEPARATION]
        block2 = cv.resize(block2, (200, 200), cv.INTER_CUBIC)
        #cv.imshow('block 2: ', block2)
        block2 = cv.cvtColor(block2, cv.COLOR_BGR2HSV)
        myHue2 = int(np.mean(block2[:,:, 0]))
        mySat2 = int(np.mean(block2[:,:, 1]))
        myVal2 = int(np.mean(block2[:,:, 2]))
        real_color2 = np.array([myHue2, mySat2, myVal2])
        if myHue1 + range >= 180:
            lowerHue1 = myHue1 - range
            upperHue1 = 180
        elif myHue1 - range <= 0:
            upperHue1 = myHue1 + range
            lowerHue1 = 0   
        else:
            lowerHue1 = myHue1 - range
            upperHue1 = myHue1 + range
            
        #make sure saturation is within range
        if mySat1 + satRange >= 255:
            upperSat1 = 255
            lowerSat1 = mySat1 - satRange
        elif mySat1 - satRange <= 0:
            upperSat1 = mySat1 + satRange
            lowerSat1 = 0
        else:
            upperSat1 = mySat1 + satRange
            lowerSat1 = mySat1 - satRange
            
        #make sure value range is within range
        if myVal1 + valRange >= 255:
            upperVal1 = 255
            lowerVal1 = myVal1 - valRange
        elif myVal1 - valRange <= BLACK_THRESH:
            upperVal1 = myVal1 + valRange
            lowerVal1 = BLACK_THRESH
        else:
            upperVal1 = myVal1 + valRange
            lowerVal1 = myVal1 - valRange
            
        if myHue2 + range >= 180:
            lowerHue2 = myHue2 - range
            upperHue2 = 180
        elif myHue2 - range <= 0:
            upperHue2 = myHue2 + range
            lowerHue2 = 0   
        else:
            lowerHue2 = myHue2 - range
            upperHue2 = myHue2 + range
            
        #make sure saturation is within range
        if mySat2 + satRange >= 255:
            upperSat2 = 255
            lowerSat2 = mySat2 - satRange
        elif mySat2 - satRange <= 0:
            upperSat2 = mySat2 + satRange
            lowerSat2 = 0
        else:
            upperSat2 = mySat2 + satRange
            lowerSat2 = mySat2 - satRange
            
        #make sure value range is within range
        if myVal2 + valRange >= 255:
            upperVal2 = 255
            lowerVal2 = myVal2 - valRange
        elif myVal2 - valRange <= 0:
            upperVal2 = myVal2 + valRange
            lowerVal2 = 0
        else:
            upperVal2 = myVal2 + valRange
            lowerVal2 = myVal2 - valRange
            
        #set colour1
        lowerColor1 = np.array([lowerHue1, lowerSat1, lowerVal1], dtype=np.uint8)
        upperColor1 = np.array([upperHue1, 255, 255], dtype=np.uint8)
        print('color 1: ', lowerHue1, upperHue1, lowerSat1, upperSat1, lowerVal1, upperVal1)
        #set colour 2
        lowerColor2 = np.array([lowerHue2, lowerSat2, lowerVal2], dtype=np.uint8)
        upperColor2 = np.array([upperHue2, 255, 255], dtype=np.uint8)
        print('color 2: ', lowerHue2, upperHue2, lowerSat2, upperSat2, lowerVal2, upperVal2)
    except Exception as e:
        exception_type, exception_object, traceback = sys.exc_info()
        line = traceback.tb_lineno
        print("Error in scanStart: ", e, "at ", line)
    
    
#uses contours to check an hsv image for colour
def CheckColour(hsv, upperCo, lowerCo):
    try:
        
        #make a mask with only our colours
        #get mask size: if it is 10 pixels it is the colour
        mask =  cv.inRange(hsv, lowerCo, upperCo) #get the part of trimmed HSV that is our colour
        #slice = cv.bitwise_and(hsv, hsv, mask = mask)
        _,mask = cv.threshold(mask, 127,255, cv.THRESH_BINARY) #black and white overlay image
        myContours = np.array([])
        myContours = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        
        for contour in myContours:
            if cv.contourArea(contour) >= 15:
                #print("shape detected", cv.contourArea(contour))
                return True
            
        return False
    #Old code
    #     #get the average h, s, v
    #     mean_h = int(np.mean(hsv[:,:,0]))
    #     mean_s = int(np.mean(hsv[:,:,1]))
    #     mean_v = int(np.mean(hsv[:,:,2]))
        
    #     avg_color = np.array([mean_h, mean_s, mean_v])
    #     # print(avg_color)
    #     # print(upperColor1)
    #     # print(lowerColor1)
        
    #     if all(avg_color <= upperCo) and all(avg_color >= lowerCo):
    #         #print("Color was within range")
    #         return True
    #     else:
    #         return False
    except Exception as e:
        #get the line
        ex_type, ex_object, traceback = sys.exc_info()
        line = traceback.tb_lineno
        print("Problem in checkColour: ", e, "at line ", line)
        return False
        
#checks both colours in image
def CheckColours(image):
    try:
        #print("in Checkcolours")
        global upperColor1
        global lowerColor1
        global upperColor2
        global lowerColor2
        #ret, image = video.read() #reads camera images as BGR (or RGB) images
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV) #use cvtColor with BGR2HSV to convert image to HSV

        #blur image
        #ksize = (3,3)
        #hsv = cv.blur(hsv, ksize)   
        #cv.imshow('Blur', hsv)  
        
        #check for both colours
        a = CheckColour(hsv, upperColor1, lowerColor1)
        b = CheckColour(hsv, upperColor2, lowerColor2)

        # print(f"a: {a}, hsv: {hsv}, upper: {upperColor1}, lower: {lowerColor1}")
        # print(f"b: {b}, hsv: {hsv}, upper: {upperColor2}, lower: {lowerColor2}")
        
        if (a):
            #print(f"detected colour a: {real_color1}")
            return tuple((cv.cvtColor(np.uint8([[real_color1]]), cv.COLOR_HSV2BGR)[0,0])) #there is a colour in that range of colour 1
        elif (b): 
            #print(f"detected colour b: {real_color2}")
            return tuple((cv.cvtColor(np.uint8([[real_color2]]), cv.COLOR_HSV2BGR)[0,0])) #there is a colour in that range of colour 2
        else:
            return (0, 0, 0) #return black
    except Exception as e:
        print("ERROR in CheckColours: ", e)
        return (181, 228, 255)


def CheckForCar(image, upperBlack, lowerBlack):
    try:
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV) #use cvtColor with BGR2HSV to convert image to HSV
        #print(lowerCo)
        return CheckColour(hsv, upperBlack, lowerBlack)
    except Exception as e:
        print("Error in checkForCars: ", e)
        return False