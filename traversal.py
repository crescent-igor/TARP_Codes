from Gps import Gps
from compass import Compass
import math
import os
from time import sleep
import numpy as nps
import socket
import serial

IP = '192.168.43.50'
PORT = 9750
BUFFER_SIZE = 1024

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

sock.connect((IP,PORT))

# =========
# Variables
# =========
nextLocationIndex = 0
# roverMotor=None                 # Doesnt fucking work
compass = None
Gps = None

# ======================
# Set locations to go to
# ======================
locations = [
   # [12.8425750, 80.1558820],
    #[12.843146, 80.1554268]
    #[12.842803, 80.156168],
    #[12.842883, 80.156212]
]

def parseCommand(bytes):
    prefix = 0xf00
    firstCommand = 0x0f0
    secondCommand = 0x00f
    x = int.from_bytes(bytes, byteorder='little', signed=False)
    
    # check prefix for base motor
    if x&prefix == 0x000:
        print('Base Motor')

        if x&firstCommand==0x040:
            print('Forward')
            return 87
        elif x&firstCommand==0x050:
            print('Backward')
            return 83
        elif x&firstCommand==0x060:
            print('Left')
            return 65
        elif x&firstCommand==0x070:
            print('Right')
            return 68
        else:
            print('Stop')
            return 27

def main():
    try:
        Gps = Gps()
        compass = Compass()
        global nextLocationIndex
        global locations
    except Exception as e:
        print(str(e))
        print("Error in creating the objects")
        return
    try:
        while 1:
            try:
                for _i in range(len(locations)):
                    Reached = False
                    while(not Reached):
                        currentLocation = Gps.getGpsData()
                        slope = getSlope(currentLocation)
                        print(centerBot(compass, slope, Gps, 10))
                        Reached = isReached(Gps)
                        if(Reached):
                            sock.send("27".encode())
                            input("Reached "+str(nextLocationIndex) +
                                  " Press anything to continue")
                        sleep(0.1)
                    nextLocationIndex += 1
            except Exception as e:
                print(str(e))
            PORT = '/dev/ttyUSB0'
            BUFFER_SIZE =  3

            ser = serial.Serial(PORT)
            command = -1
            ignoreCommand = False
            x = -1
            toAngle = 0

            try:
                while True:
                    try:
                        data = ser.read(BUFFER_SIZE)
                        currentAngle = compass.getCompassAngle()
                        if(not ignoreCommand):
                            x=parseCommand(data)
                            print("\n")
                            if(x==27):
                                toAngle = currentAngle
                                print("Stop")
                            elif(x==87):
                                toAngle = currentAngle
                                print("Forward")
                            elif(x==65):
                                toAngle = (currentAngle - 90 + 360)%360
                                ignoreCommand = True
                                print("Left")
                            elif(x==68):
                                toAngle = (currentAngle + 90 + 360)%360
                                ignoreCommand = True
                                print("Right")
                            else:
                                print("Command Not Found ")
                        sock.send(str(x).encode())
                        os.system(cmd)
                    except Exception as e:
                        print(e)
                        ser.close()
                        ser = serial.Serial(PORT)
            except Exception as e:
                print(e) 
            finally:
                ser.close()
    except Exception as e:
        print(str(e))
    finally:
        global sock
        sock.close()

# Returns the angle to move the rover to


def getSlope(currentLocation):
    global nextLocationIndex
    # global scaledLocations
    global locations
    y1 = currentLocation[0]
    x1 = currentLocation[1]
    y2 = locations[nextLocationIndex][0]
    x2 = locations[nextLocationIndex][1]
    # print(locations[nextLocationIndex], end=" ")

    try:
        slope = math.atan((y2-y1)/(x2-x1))*180/(math.pi)
#                print("tan value"+str(slope))
        if(x1 > x2 and y1 > y2):
            slope = 180-slope
        elif(x1 < x2 and y1 > y2):
            slope = -slope
        elif(x1 > x2 and y1 < y2):
            slope = 180-slope
        elif(x1 < x2 and y1 < y2):
            slope = 360-slope
        elif(slope==0):
            if(x1>x2):
                slope=180    

    except ZeroDivisionError:
        if(y1>y2):
            slope=90roverGps
RoverGps
roverGps
roverGPS
RoverGps
roverGPS
roverGPS
roverGPS
roverGps
roverGps
roverGps
roverGps
RoverGps
roverGps
        else:
            slope=270
        # print("\n\nDivide by 0")
        # return

    return slope

def centerBot(compass, slope, Gps, threshold=5):
    global sock
    currentLocation = Gps.getGpsData()
    try:
        angle = compass.getCompassAngle()
    except ValueError as e:
        print("\nError: Inside centerBot: Calibrate Compass")
        return False
    except Exception as e:
        print(str(e))
        return False
    try:
        #                print(angle, getAngle)
        angle = float(str(angle))
        print(angle, slope)

        if(slope+threshold > 360):
            if(angle < 180):
                if(angle > (slope+threshold-360)):

                    print("Centering Rover!", currentLocation,
                          locations[nextLocationIndex], end=" : ")
                    print("Rotate left ")
                    #global sock
                    sock.send("65".encode())
                    #os.system("python demo.py 65")
                    return False
                else:
                    #global sock
                    sock.send("87".encode())
                    #os.system("python demo.py 87")
                    return True

            else:
                if(angle < (slope-threshold)):
                    print("Centering Rover!", currentLocation,
                          locations[nextLocationIndex], end=" : ")
                    print("Rotate Right")
                    #global sock
                    sock.send("68".encode())
                    #os.system("python demo.py 68")
                    return False
                else:
                    #global sock
                    sock.send("87".encode())
                    #os.system("python demo.py 87")
                    return True
        elif(slope-threshold < 0):
            if(angle < 180):
                if(angle > slope+threshold):
                    print("Centering Rover!", currentLocation,
                          locations[nextLocationIndex], end=" : ")
                    print("Rotate Left")
                    #global sock
                    sock.send("65".encode())
                    #os.system("python demo.py 65")
                    return False
                else:
                    #global sock
                    sock.send("87".encode())
                    #os.system("python demo.py 87")
                    return True
            else:
                if(angle < (360-(threshold-slope))):
                    print("Centering Rover!", currentLocation,
                          locations[nextLocationIndex], end=" : ")
                    print("Rotate right")
                    #global sock
                    sock.send("68".encode())
                    #os.system("python demo.py 68")
                    return False
                else:
                    #global sock
                    sock.send("87".encode())
                    #os.system("python demo.py 87")
                    return True
        else:
            if(angle < slope-threshold):
                print("Centering Rover!", currentLocation,
                      locations[nextLocationIndex], end=" : ")
                print("Rotate right")
                #global sock
                sock.send("68".encode())
                #os.system("python demo.py 68")
                return False
            elif(angle > slope+threshold):
                print("Centering Rover!", currentLocation,
                      locations[nextLocationIndex], end=" : ")
                print("Rotate Left")
                #global sock
                sock.send("65".encode())
                #os.system("python demo.py 65")
                return False
            else:
                #global sock
                sock.send("87".encode())
                #os.system("python demo.py 87")
                return True
        #     print("Centering Rover!", currentLocation,
        #           locations[nextLocationIndex], end=" : ")
        #     if(angle > getAngle):
        #         print("Rotate left ", getAngle, angle)
        #         # roverMotor.moveMotor('right')
        #     else:
        #         print("Rotate right ", getAngle, angle)
        #         # roverMotor.moveMotor('left')
        #     return False
        # else:
        #     return True
    except Exception as e:
        print("TypeError in centerBot")
        print(str(e))
        return False


def isReached(Gps, bound=0.00002):
    global locations, nextLocationIndex
    bufferLen = 1
    result = True
#        Gps=Gps()
    while(bufferLen > 0):
        #sleep(0.1)
        currentLocation = Gps.getGpsData()
        if(abs(currentLocation[0]-locations[nextLocationIndex][0]) < bound and abs(currentLocation[1]-locations[nextLocationIndex][1]) < bound):
            # print(abs(currentLocation[0]-locations[nextLocationIndex][0]),abs(currentLocation[1]-locations[nextLocationIndex][1]))
            result = result and True
        else:
            result = False
        bufferLen = bufferLen-1
    print(abs(currentLocation[0]-locations[nextLocationIndex][0]),
          abs(currentLocation[1]-locations[nextLocationIndex][1]))
    return result


if __name__ == "__main__":
    main()
