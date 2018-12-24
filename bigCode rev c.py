#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 22 21:25:02 2018

@author: samchili

some authors say you need the waitkey before it shows a window
import cv2
img = cv2.imread("image.jpg")
cv2.namedWindow("preview")
cv2.imshow("preview", img)
cv2.waitKey()
or you can use cv2.startWindowThread()
https://txt.arboreus.com/2012/07/11/highgui-opencv-window-from-ipython.html
https://www.programcreek.com/python/example/89352/cv2.startWindowThread


"""

import cv2
import serial
import math
import numpy as np
import time
import requests
import datetime
import json
lasttime = datetime.datetime.now()

#HEARTBEAT################################################################################################
#ipAddress = 192.168.65.2 # course A
ipAddress = '192.168.66.2' # course B
#ipAddress = 192.168.67.2 # course C
#ipAddress = 'ec2-54-89-60-172.compute-1.amazonaws.com' #test server
ipPort = 8080
#testCourse = 'testCourse9'
#testCourse = 'courseA'
testCourse = 'courseB'
#testCourse = 'courseC'

lastGpsLat=float(0.0)
lastGpsLon = float(0.0)

speedLat = 0.0
speedLon = 0.0


def heartbeat(ip, port, course, challenge, lat, long, protocol = "http", teamCode = "NHHS"):
    
    url = "{protocol}://{ip}:{port}/hearbeat/{course}/{teamCode}".format(protocol = protocol, ip = ip, port = port, course = course, teamCode = teamCode)

    print('sending heartbeat')
   
    headers = {'content-type' : 'application/json'}
    
    timestamp = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
    print(timestamp)
    
    #dictionary
#    lib = {"timestamp" : timestamp,
#           "challenge" : challenge,
#           "position" :{"datum" : "WGS84" , "latitude" : lat , "longitude" : long}}
    lib = {"timestamp" : timestamp,
           "challenge" : challenge,
           "position" :{"datum" : "WGS84" , "latitude" : lastGpsLat , "longitude" : lastGpsLon}}
    
    
    
    PARAMS = json.dumps(lib)
    print(PARAMS)
    resp = requests.post(url = url , headers = headers, data = PARAMS)
    
    #convert data to json format
    #data = resp.json()
    
    return resp.text

def StartRun(ip, port, course, protocol = "http", teamCode = "NHHS"):
    
    url = "{protocol}://{ip}:{port}/run/start/{course}/{teamCode}".format(protocol = protocol, ip = ip, port = port, course = course, teamCode = teamCode)

    print('Sending Start Run message')
    
    resp = requests.post(url = url )#, headers = headers, data = PARAMS)
#    print resp.text
#    
#    if(resp['success']== True)
#        return True
#    else
    return resp.text
def StopRun(ip, port, course, protocol = "http", teamCode = "NHHS"):
    
    url = "{protocol}://{ip}:{port}/run/end/{course}/{teamCode}".format(protocol = protocol, ip = ip, port = port, course = course, teamCode = teamCode)
    
    print('Sending Stop Run message')
    
    resp = requests.post(url = url )#, headers = headers, data = PARAMS)
    
    data = resp.json()
    
    return data

#resp1 = StartRun('ec2-54-89-60-172.compute-1.amazonaws.com',8080,'testCourse1')
#print(resp1)
#
#for i in range( 1,10):
#    heartbeat1Resp = heartbeat('ec2-54-89-60-172.compute-1.amazonaws.com' , 8080 , 'testCourse1' , 'speed' , 40.689249 , -74.044500)
#    print(heartbeat1Resp)
#    time.sleep(1)
#    
#resp2 = StopRun('ec2-54-89-60-172.compute-1.amazonaws.com',8080,'testCourse1')
#print(resp2)


######## Everything but heartbeat
ser = serial.Serial()
ser.baudrate = 115200
ser.port = '/dev/ttyACM0'#subject to change
ser.timeout = 0
ser.open()

function = 1
cap = cv2.VideoCapture(0)
kernel = np.ones((5,5), np.uint8)
positionError = 0.00003      #1 degree is 69 miles, so 1e-6*1degree is .34feet, so .0000030 is about 10 feet               #3, 4, 5, 6?
motorSpeed = "20"
compassError = 5
heightMinimum = 100
height = 0
width = 0


def positionCheck(x, lowerThresh, higherThresh):        #check y value as well?
    if(x > higherThresh or x < lowerThresh):
        return(False)
        
    elif(x > lowerThresh and x < higherThresh):
        return(True)
def findColor(color, ar, tol):
    ret, img = cap.read()
    #img1=img
    #img = img[134:480, 0:640]
    
    if ret == False:
        return(-1)
    #img = img[310:713, 0:555]
#CONVERT BGR TO LAB
    lab= cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    cv2.imshow("lab",lab)
#SPLIT LAB CHANELS
    l, a, b = cv2.split(lab)
    cv2.imshow('l_channel', l)
    cv2.imshow('a_channel', a)
    cv2.imshow('b_channel', b)
    ainv = cv2.bitwise_not(a)
    binv = cv2.bitwise_not(b)
    linv = cv2.bitwise_not(l)
    linv = linv
    cv2.imshow('ainv',ainv)
    
    if(color == "red"):
        e = a
        threshh = 150
    elif(color == "green"):
        e = ainv
        threshh = 140
    elif(color == "blue"):
        e = binv
        threshh = 145
        
    blur = cv2.bilateralFilter(e,9,75,75)
    blur = cv2.GaussianBlur(blur,(5,5),0)
    _,thresh = cv2.threshold(blur,threshh,255,cv2.THRESH_BINARY)#140, 255
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    #cv2.imshow('thresh',opening)
    
    x = 0
    _,contours, h = cv2.findContours(opening,1,2)
    maximum = 0
    index = 0
    resultR = []
    for j in range(len(contours)):
        x, y, w, h = cv2.boundingRect(contours[j])
        if(h/w >= ar-tol and h/w <= ar+tol):                          #2 and 8 or something else?
            resultR.append(contours[j])
    for i in range(len(resultR)):
        x,y,w,h = cv2.boundingRect(resultR[i])
        if(maximum < y+h and h >= heightMinimum):                     #100 or more or less?
            maximum = y+h
            index = i
            height = h
            print(height)
    if len(resultR) != 0:  
            c = resultR[index]
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)       
                
    return(x)
       
def goToLocation(curLat, curLon, goalLat, goalLon): #function call will have curGPS() as args
    curLat = float(curLat)
    curLon = float(curLon)
    goalLat = float(goalLat)
    goalLon = float(goalLon)
    latDif = goalLat - curLat
    lonDif = goalLon - curLon
    
    heading = (math.atan2(lonDif, latDif) * 180) / 3.14159265359
    print(goalLat, goalLon, curLat, curLon, heading)

    currentHeading = int(getCompass())
    error = heading - currentHeading
    direction = 0
    buoyr = findColor("red", 4, 1, 1)       #jfa - ok, so dodge left if it's red, and dodge right if it's green
    buoyg = findColor("green", 4, 1, 1)
    if(positionCheck(buoyr, 280, 340)):         
        SerCheck('[MOR' + motorSpeed + ']')
        SerCheck('[MOL0]')
        print('left')
    if(positionCheck(buoyg, 280, 360)):     #jfa - these both might be true, should this be elif?
        print("turn rigt")
        SerCheck('[MOL' + motorSpeed + ']')
        SerCheck('[MOR0]')
    else:
        if(error > 0):
            direction = 1                   #jfa - maybe just plunk in gotoheading here?
        elif(error < 0):
            direction = -1
        if(abs(error) >= 180):
            direction *= -1
        
        if(abs(error) <= compassError):
            SerCheck('[MOL' + motorSpeed + ']')
            SerCheck('[MOR' + motorSpeed + ']')
            print('forward')
        elif(direction > 0):
            SerCheck('[MOL' + motorSpeed + ']')
            SerCheck('[MOR-10]')
            print('right')
        elif(direction < 0):
            SerCheck('[MOR' + motorSpeed + ']')
            SerCheck('[MOL-10]')
            print('left')
    
def whileGoToLocation(curLat, curLon, goalLat, goalLon):
    while(youveArrived(curLat, curLon, goalLat, goalLon) == False):
        goToLocation(curLat, curLon, goalLat, goalLon)
          
def youveArrived(curLat, curLon, goalLat, goalLon):
    if abs(curLat - goalLat) < positionError:
        if abs(curLon - goalLon) < positionError:
            print(abs(curLat - goalLat), abs(curLon - goalLon))
            return(True)
        else:
            return(False)
    else:
        return(False)

def getCompass():
    valid, echo = SerCheck('[CO?]')
    if valid and len(echo) > 4:
        my_list = echo.decode().split(',')
        heading = my_list[1]
        print(heading)
        return(heading)
    else:
        print('not valid')
        return(-1)

def getGPS():
    valid, response = SerCheck('[GP?]')
    if valid:
        my_list = response.decode().split(',')
        print(my_list)
        if len(my_list) >= 4:
            lastGpsLat = float(my_list[2])/1000000.
            lastGpsLon = float(my_list[3])/1000000.
            return (float(my_list[2])/1000000, float(my_list[3])/1000000)
        else:
            return('0','0')
    else:
        return('0','0')
        
def goToHeading(curCompass, goalCompass):
    #do stuff to maintain same heading
    #no return
    if(goalCompass >= 360):
        goalCompass -= 360
    if(curCompass > -1):
        if abs(curCompass - goalCompass) < compassError:
                SerCheck('[MOL' + motorSpeed + ']')
                SerCheck('[MOR' + motorSpeed + ']')
                print('forward')
                return(1)
        else:
            if curCompass > goalCompass:
                SerCheck('[MOR' + motorSpeed + ']')
                SerCheck('[MOL0]')
                print('left')
                
            elif curCompass < goalCompass:
                SerCheck('[MOL' + motorSpeed + ']')
                SerCheck('[MOR0]')
                print('right')
            return(0)
            
def SerCheck(steve):
    if ser.isOpen()==False:
        ser.open()
    if ser.isOpen()== True:
        ser.write(steve.encode())
        time.sleep(0.01)
        ser.flush()
        response = ser.read(ser.inWaiting())
        print(1, response)
        if len(response) > 1:
            return(1, response)
        else:
            return(0, "")
        time.sleep(0.01)
    else:
        return(0, '')

def theGate():
    count = 0
    while(1):
        getGPS()
        heartbeat(ipAddress, 8080, testCourse, "auto", lastGpsLat, lastGpsLon)
        lab= cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    #SPLIT LAB CHANELS
        l, a, b = cv2.split(lab)
        cv2.imshow('l_channel', l)
        cv2.imshow('a_channel', a)
        cv2.imshow('b_channel', b)
        ainv = cv2.bitwise_not(a)
        cv2.imshow('ainv',ainv)
    #BLUR RED IMAGE
        blur = cv2.bilateralFilter(a,9,75,75)
        blur = cv2.GaussianBlur(blur,(5,5),0)
    #BLUR GREEN IMAGE
        blurinv = cv2.bilateralFilter(ainv,9,75,75)
        blurinv = cv2.GaussianBlur(blurinv,(5,5),0)
    #THRESHOLDING RED
        _,thresh = cv2.threshold(blur,150,255,cv2.THRESH_BINARY)
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        cv2.imshow('thresh',opening)
    
    #THRESHOLDING GREEN
        _,threshinv = cv2.threshold(blurinv,140,255,cv2.THRESH_BINARY)
        openinginv = cv2.morphologyEx(threshinv, cv2.MORPH_OPEN, kernel)
        cv2.imshow('threshinv',openinginv)
    #COMBINE RED AND GREEN IMAGES
        stuff = opening + openinginv
        cv2.imshow('stuff',stuff)
        
    #RED CONTOURS AND BOX
        _,contours, h = cv2.findContours(opening,1,2)
        height = 0
        heightG = 0
        if len(contours) != 0:
            
            maximum = 0
            index = 0
            resultR = []
            for j in range(len(contours)):
                x, y, w, h = cv2.boundingRect(contours[j])
                print("main while loop ", w/h, h/w)
                print("height", h)
                if(h/w >= 2 and h/w <= 5):
                              #2 and 8 or something else?
                    resultR.append(contours[j])
            for i in range(len(resultR)):
                x,y,w,h = cv2.boundingRect(resultR[i])
                if(maximum < y+h):                     #100 or more or less?
                    maximum = y+h
                    index = i
                    height = h
                    print(height)
    #GREEN CONTOURS AND BOX
        resultG = []
        _,contoursG, hG = cv2.findContours(openinginv,1,2)
        if len(contoursG) != 0:
            maximumG = 0
            indexG = 0
            resultG = []
            for j in range(len(contoursG)):
                xg, yg, wg, hg = cv2.boundingRect(contoursG[j])
                if(hg/wg >= 2 and hg/wg <= 5):                          #2 and 8 or something else?
                    resultG.append(contoursG[j])
            for i in range(len(resultG)):
                xg,yg,wg,hg = cv2.boundingRect(resultG[i])
                if(maximumG < yg+hg):                     #100 or more or less?
                    maximumG = yg+hg
                    indexG = i
                    heightG = hg
                    print(heightG)
        if len(resultR) != 0:  
                c = resultR[index]
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)       
                    
        if len(resultG) != 0:  
                cg = resultG[indexG]
                xG,yG,wG,hG = cv2.boundingRect(cg)
                cv2.rectangle(img,(xG,yG),(xG+wG,yG+hG),(0,255,0),2)
                
        distance = (xG-(x+w)) #gx - x
        mid = (x+w) + (distance / 2)
        
        centerBuoy = 320
        toleranceBuoy = 100
    
        if len(contours) == 0 and len(contoursG) == 0:
    #FINAL IMAGE WITH BOXES
            print("turning in circle")
            SerCheck('[MOL20]')
            SerCheck('[MOR20]')
            if(count > 1):                 
                break
        else:  
            count += 1
            if (mid >= centerBuoy-toleranceBuoy) and (mid <= centerBuoy+toleranceBuoy):
                print('go forward')
                SerCheck('[MOL30]')
                SerCheck('[MOR30]')
            elif mid < centerBuoy-toleranceBuoy:
                print('turn left')
                SerCheck('[MOL10]')
                SerCheck('[MOR30]')
            elif mid > centerBuoy+toleranceBuoy:
                print('turn right')
                SerCheck('[MOL30]')
                SerCheck('[MOR10]')
        cv2.imshow('img',img)
        
        if(cv2.waitKey(1) & 0xFF == ord('9')):#wait key of 9 to break while loops?
            break
        
           
def speedGate():
    curCompass = 10
    #go to way point
    curLat, curLon = getGPS()
    whileGoToLocation(curLat, curLon, speedLat, speedLon)
    #line up the red and green until they're in certain part of the screen
    red = findColor("red", 1, 0.5)
    green = findColor("green", 1, 0.5)
    if(red > 0 and green > 0):
        #while(red > 250):# or green < 350
        if(red > 250):
            print("right")
            SerCheck('[MOL'+ motorSpeed + ']')
            SerCheck('[MOR0]')
#        elif(green < 350):
#            print("left")
#            SerCheck('[MOR'+ motorSpeed + ']')
#            SerCheck('[MOL0]')
        else:
            break
    else:
        print("turn in slow circle")
        SerCheck('[MOL-10]')
        SerCheck('[MOR10]')
    curCompass = int(getCompass())
    goalCompass = curCompass + 181
    valid, startLat, startLon = getGPS()    
    #blueBuoyCount = 0
    while(1):                               #we don't see the blue buoy, so just go ahead until we do and hope for the best
        ret, img = cap.read()
        cv2.imshow("img", img)
    #    if(ret == False):
    #        print("broken camera in first while loop")
    #        break
        blueBuoy = findColor("blue", 1, 0.5)
        #updateCamera()
        if(blueBuoy <= 0):
            goToHeading(curCompass, curCompass)
        elif(blueBuoy > 0):
                break
    
    while(1):                               #while we see the blue buoy, sprial in on it by keeping constant angle
        ret, img = cap.read()
        if(ret == False):
            print("broken camera")
            break
        cv2.imshow("img", img)
        blueBuoy = findColor("blue", 1, 0.5)
        #updateCamera()
        if(blueBuoy > 0):
            #curCompass = getCompass()
            if(height > 50):
                print(height)
                break
            #do stuff with fps cooridinates for minute minute
            elif(blueBuoy >= 100 and blueBuoy <= 150):
                SerCheck('[MOL'+ motorSpeed + ']')
                SerCheck('[MOL'+ motorSpeed + ']')
                print("looking for blue forward")
            elif(blueBuoy < 100):
                SerCheck('[MOR'+ motorSpeed + ']')
                SerCheck('[MOL0]')
                print("looking for blue left")
            else:
                SerCheck('[MOL'+ motorSpeed + ']')
                SerCheck('[MOR0]')
                print("looking for blue right")
        else: # we've lost track of the blue buoy
            curCompass = int(getCompass())      #so keep going around left until we get all the way around
            if(curCompass <= goalCompass):
                break
            SerCheck('[MOR'+ motorSpeed + ']')
            SerCheck('[MOL0]')
    
    while(1):                                   #go back to original gps coordinates
        ret, img = cap.read()
        if(ret == False):
            print("broken camera")
            break
        print("return home while loop")
        curLat, curLon = getGPS()
        goToLocation(curLat, curLon, startLat, startLon)
        #updateCamera()
        if(youveArrived(curLat, curLon, startLat, startLon)):
            print("arrived")
            break

def followTheLeader():
    while(1):
        red = findColor("red", 1, 0.25)
        if(red > 0):
            #WITH HEIGHT
            if(height > 250):
                print("stop moving")
                SerCheck('[MOL0]')
                SerCheck('[MOR0]')
            elif(height < 200):
                print("speed up")
            else:
                print("go forward at same speed")
            
            #WITH PIXELS
            if(red >= 100 and red <= 200):
                print("go forward")
                SerCheck('[MOL' + motorSpeed + ']')
                SerCheck('[MOR' + motorSpeed + ']')
            elif(red > 200):
                print("turn right")
            else:
                print("turn left")


while(1):
    ret, img = cap.read()                       #jfa - so we also need a way to quit a program that's not working and go back to the dock, and that'll put a waitkey in where it might make the imshow work?
    if(ret):
        if(cv2.waitKey(1) & 0xFF == ord('1')):
            StartRun(ipAddress, 8080, testCourse)
            theGate()
        elif(cv2.waitKey(1) & 0xFF == ord('2')):
            speedGate()
        elif(cv2.waitKey(1) & 0xFF == ord('3')):
            followTheLeader()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
StopRun(ipAddress, 8080, testCourse)
cap.release()
cv2.destroyAllWindows()
ser.close()   
        