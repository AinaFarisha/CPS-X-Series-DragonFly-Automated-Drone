#!/usr/bin/env python
# NOTE: Line numbers of this example are referenced in the user guide.
# Don't forget to update the user guide after every modification of this example.
# sudo systemctl start firmwared
# sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone::stolen_interface=::simple_front_cam=true
# source ~/code/parrot-groundsdk/./products/olympe/linux/env/shell
# python ./dronewebcam.py


import csv
import cv2
import math
import os
import queue
import shlex
import subprocess
import tempfile
import threading
import traceback
from anafiRequestPost import Anafi_Request_Post
from anafiScanning import Anafi_Scanning
import time
from pyzbar import pyzbar
import PySimpleGUI as sg
import numpy as np
from PIL import Image
import io

import olympe
import olympe_deps as od
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.PilotingEvent import moveByEnd
from olympe.messages.ardrone3.Piloting import moveBy, CancelMoveBy
from olympe.messages.ardrone3.Piloting import moveTo
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, AltitudeChanged, GpsLocationChanged, PositionChanged, moveByChanged, AltitudeAboveGroundChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.common.CommonState import BatteryStateChanged
from olympe.messages.move import extended_move_by
from olympe.enums.ardrone3.PilotingState import FlyingStateChanged_State

from olympe.messages.camera import max_zoom_speed

from olympe.enums.camera import zoom_control_mode
from olympe.enums.camera import white_balance_temperature
from olympe.messages.camera import set_zoom_target
from olympe.messages.camera import set_white_balance
from olympe.messages.camera import zoom_level
olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

DRONE_IP = "10.202.0.1"
RACK_LEVEL = 3

class AnafiConnection(threading.Thread):
 
    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(DRONE_IP)

        # webcam 
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # FPS = 1/X
        # X = desired FPS
        self.FPS = 1/30
        self.FPS_MS = int(self.FPS * 1000)
        self.recording = False

        sg.theme('Black')
        self.layout = [
            #   [sg.Text('OpenCV Demo', size=(40, 1), justification='center', font='Helvetica 20')],
              [sg.Image(filename='', key='image')],
              [sg.Button('Start Streaming', size=(10, 1), font='Helvetica 12'),
               sg.Button('End Streaming', size=(10, 1), font='Helvetica 12'),
               sg.Button('Start Operation', size=(10, 1), font='Helvetica 12'),
               sg.Button('End Operation', size=(10, 1), font='Helvetica 12'),
               sg.Button('Exit', size=(10, 1), font='Helvetica 12'), ],
               [sg.Text('Zoom Level: ', size=(10, 1), justification='center', font='Helvetica 12'), sg.Slider(range=(1.0,3.0), key='zoomlevel', default_value=1.0, resolution=.1, size=(50,15), orientation='horizontal')]]

        self.window = sg.Window('Anafi Video Streaming',
                       self.layout, location=(600, 200))


         # initialize the known distance from the camera to the object (referring to picture)
        self.KNOWN_DISTANCE = 25.1

        # initialize the known object width, which in this case, the barcode(referring to QR code in picture)
        self.KNOWN_WIDTH = 2.16535

        # change path to your own path
        self.image = cv2.imread("/home/dragonfly/aina/physical_parrot/images/barcode.jpg")

        # decode the QR code in the picture
        self.foundBarcode = pyzbar.decode(self.image)

        # loop the found barcode and get the width (in pixels) for the QR code in the picture, then calculate the focal length
        for QRCode in self.foundBarcode:
            (x, y, w, h) = QRCode.rect
        self.focalLength = (w * self.KNOWN_DISTANCE) / self.KNOWN_WIDTH

        self.request_post = Anafi_Request_Post()
        self.scanning_decode = Anafi_Scanning()

        # comment/uncomment this part if want to read location from txt file
        self.listOfLocation = self.request_post.readLocation()

        # comment/uncomment this part if want to read location from server
        # self.listOfLocation = self.request_post.getLocation()
        
        # flag for the current location and status
        self.currentLocation = None
        self.currentLocationStatus = False

        
        # intructions related
        self.currentInstruction = "NONE"
        self.rackInstructions = ["UP", "DOWN", "RIGHT", "LEFT", "END"]
        self.inst = None

        self.cr = 0


        # rack list 1 (Odd/Ganjil) 
        self.oddLocation = None

        # rack list 2 (even/Genap)
        self.evenLocation = None

        
        self.rackNumber = 0
        # rack related
        self.level = 0
        self.actualRackLevel = RACK_LEVEL #change based on situation

        # initializing the barcode data list for storing the list of barcode that will be scanned later
        self.barcodeDataList = []

        # initializing for forward and backward movement to avoid crash
        self.forwardBackward = 0
        
        # current scanned barcode data
        self.barcodeData = ""

        # to start scanning operation. Only use this when scanning 4 penjuru
        # use at scanning up instruction
        self.moveUpCounter = 0
        
        # how many operation has been done
        self.operationCalculationCounter = 0

        """Distance Related"""
        #for distance x and distance y for scanning area
        self.final_dX = 0.0
        self.final_dY = 0.0
        
        # to check the location after scanning all items
        self.lastX_Position = self.lastY_Position = 0

        # default position before scanning rack
        self.dY_leftBottom_afterTakeoff = self.dX_leftTop = self.dY_leftTop = self.dX_rightTop = self.dY_rightTop = self.dX_rightBottom = self.dY_rightBottom = self.dX_leftBottom = self.dY_leftBottom = None
        
        # maximum distance for drone to move 
        self.limitDistance = 3
        
        # use for testing
        # self.dY_leftBottom_afterTakeoff =  self.dX_leftTop = self.dX_rightBottom = self.dY_rightBottom = 0
        # # self.dY_leftTop = self.dX_rightTop = self.dY_rightTop = self.dY_leftBottom = 3  
        # self.dX_leftBottom = None
        
        # self.dX_leftTop = 0
        # self.dX_rightTop = 2
        # self.dY_leftBottom = 0
        # self.dY_leftTop = 3
        # self.dY_rightTop = 3
        # test until here
        

        self.warningSignal = None
        # self.forceLandingSignal = False
        """distance for calculation"""
        self.X0 = None
        self.X1 = None
        self.Y0 = None
        self.Y1 = None
        self.Y2 = None

        self.cr1 = None
        self.cr2 = None
        self.cr3 = None
        
        """flag signal"""
        self.drone_in_operation = True
        self.scanRackSignal = True
        self.scanItemSignal = False
        self.changeRackSignal = False
        self.zoomLevel = 1.0
        self.event = None
        self.values = None

        super().__init__()
        print("Initialization succesfull, Drone is ready to FLY")
        super().start()

    
    def start(self):
        # Connect the the drone
        try:
            
            self.drone.connect()
            if self.drone.connect():
                print("**----------> Drone Connected")
                print("**----------> Battery percentage before takeoff:", self.drone.get_state(BatteryStateChanged)["percent"])
                print("**----------> Altitude before takeoff(should be 0):", self.drone.get_state(AltitudeChanged)["altitude"])

                # camera zoom 2x
                # self.drone(
                # set_zoom_target(
                #     cam_id=0,
                #     control_mode=0,
                #     target=2.0
                #     )
                # ) 
                #camera white balance
                # self.drone(set_white_balance(cam_id=0, mode=16, temperature=1))
                # Start video streaming
                self.drone.start_video_streaming()
                
            else:
                print("**---------->[ERROR] Drone NOT CONNECTED [ERROR]<----------**")
        except:
            print("Error at starting")

    # Properly stop the video stream and disconnect
    def stop(self):
        self.drone.disconnect()

    def show_yuv_frame(self, window_name, yuv_frame):
        # x = round(0.5)
        cv2frame = yuv_frame

        # cv2frame = Image.fromarray(yuv_frame)  # create PIL image from frame
        # bio = io.BytesIO()  # a binary memory resident stream
        # cv2frame.save(bio, format= 'PNG')  # save image as png to it
        # imgbytes = bio.getvalue()  # this can be used by OpenCV hopefully
        self.zoomLevel = self.values['zoomlevel']
        self.drone(
            set_zoom_target(
                cam_id=0,
                control_mode=0,
                target=self.zoomLevel
                )
            ) 

        zoomlevel = str(self.zoomLevel) + "x" 

        cv2.putText(cv2frame, zoomlevel, (cv2frame.shape[1] - 60, cv2frame.shape[0] - 450), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # shows battery percentage on camera screen
        batteryPercentage = self.drone.get_state(BatteryStateChanged)["percent"]
        batteryText = "Battery Level: " + str(batteryPercentage) + "%"
        # print("cv2frame.shape[1] ",cv2frame.shape[1])
        # print("cv2frame.shape[0] ",cv2frame.shape[0])
        if (batteryPercentage < 25):
            self.warningSignal = 3
            cv2.putText(cv2frame, batteryText, (cv2frame.shape[1] - 620, cv2frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            cv2.putText(cv2frame, batteryText, (cv2frame.shape[1] - 630, cv2frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        """Display only when drone is not landed"""

        # if self.drone.get_state(FlyingStateChanged)["state"] == FlyingStateChanged_State.landed :

        if (self.warningSignal != None) :
            # 1 = Force Landing 2 = Force Backoff and landing  3 = Battery low
            # warningText = "WARNING!!! "

            if (self.warningSignal == 1):
                warningText = "WARNING!!! FORCE LANDING"
                cv2.putText(cv2frame, warningText, (cv2frame.shape[1] - 1250, cv2frame.shape[0] - 650), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 255), 3)
            
            elif (self.warningSignal == 2):
                warningText = "WARNING!!! FORCE BACKOFF AND LANDING"
                cv2.putText(cv2frame, warningText, (cv2frame.shape[1] - 1250, cv2frame.shape[0] - 650), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 255), 3)
            
            elif (self.warningSignal == 3):
                warningText = "WARNING!!! BATTERY LOW"
                cv2.putText(cv2frame, warningText, (cv2frame.shape[1] - 1250, cv2frame.shape[0] - 650), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 255), 3)
            
            else:
                pass

        # scan the barcode, draw box and data in the frame
        self.barcodeInfo  = self.scanning_decode.startScanning(cv2frame, self.focalLength, self.KNOWN_WIDTH)

        if not self.barcodeInfo:
            pass

        else:
        #  str(self.barcodeInfo) != "None":
            # print("Area", self.barcodeInfo[1][1])
            # forward backward to avoid crash
            # print("self.barcodeInfo[1]: ", self.barcodeInfo[1])
            distance = self.barcodeInfo[1]
            self.forwardBackward = self.avoidCrash_usingDistance(float(distance)) #//boleh amik nnt kalau nak guna 
            # print(self.forwardBackward)
            
            self.barcodeData1 = str(self.barcodeInfo[0])
            # print(self.barcodeData)

            # to read the tags only once at a time 
            if self.barcodeData1 != self.barcodeData:
                self.barcodeData = str(self.barcodeInfo[0])
                codeInfo= str(self.barcodeData).split()
                print("codeInfo: ", codeInfo[0])

                if len(codeInfo) > 1: 

                    if (self.changeRackSignal == True) and (codeInfo[0] == "CR") :
                        self.cr1 = codeInfo[1]
                        self.cr2 = codeInfo[2]
                        self.cr3 = codeInfo[3]
                        # self.changeRackSignal = False
                        # self.scanItemSignal = True
                        # self.land() #use to test
                        print("latest change rack signal: ", self.changeRackSignal)
                        print("latest scan item signal: ", self.scanItemSignal)

                    else:
                        self.storeData(codeInfo[0]) # store rack number or data
                        self.currentInstruction = codeInfo[1]
                        print(">>>>>current movement instruction: ", self.currentInstruction)
                        print(">>>>>distance between code and drone: ", distance)
                        print(">>>>>forwardBackward collision: ", self.forwardBackward)

                else:
                    self.storeData(codeInfo[0])
                    print(">>>>>distance between code and drone: ", distance)
                    print(">>>>>forwardBackward collision: ", self.forwardBackward)
        
        imgbytes = cv2.imencode('.png', cv2frame)[1].tobytes()
        # Use OpenCV to show this frame
        # cv2.imshow(window_name, cv2frame)
        # cv2.waitKey(1)  # please OpenCV for 1 ms...
        # imgbytes = cv2.imencode('.png', cv2frame)[1].tobytes()  # ditto
        # self.window['image'].update(cv2frame)
        self.window['image'].Update(data=imgbytes)

    def scanningItemMovement(self, count, y, x):

        count2 = count
        two_y = y*2
        print("**----------> distance x that drone should travel: ", x)
        print("**----------> distance y that drone shoud travel: ", y)
        
        # if self.rackNumber == 1: 
        #     """ go to middle of first level"""
        #     print("go to middle of first level")
        #     goToFirstLevel_Y = self.final_dY - y
        #     self.cancelmove()
        #     print(-x, " ",goToFirstLevel_Y)
        #     self.move_Down2(-x, goToFirstLevel_Y)
        # else:
        #     pass
        
        if count2%2 == 0:
            countType = "Even Number of Rack Level"
            print(countType)

            while count > 0:
         
                print("count is: ", count)
                self.level += 1

                if count%2 == 0:
                    self.cancelmove()
                    self.move_Right2(x)
                    self.cancelmove()

                elif count%2 == 1:
                    self.cancelmove()
                    self.move_Left2(x)
                    self.cancelmove()

                    # self.lastY_Position= round((self.drone.get_state(AltitudeChanged)["altitude"]), 2)
                    # self.lastX_Position = round((self.drone.get_state(moveByEnd)["dY"]), 2)
                    # print("Last Location X1: ", self.lastX_Position) # X is expected to be negative value
                    # print("Last Location Y1: ", self.lastY_Position)
                    self.lastX_Position = -1
                    print("Last Location X1: ", self.lastX_Position) 
                
                count = count-1
                if count != 0:
                    self.move_Up2(two_y)

            else:
                print("---------------->>>> DONE SCANNING RACK")
                self.scanItemSignal = False
                self.changeRackSignal = True

        elif count2 %2 == 1:
            countType = "Odd Number of Rack Level"
            print(countType)

            while count > 0:
                print("count is: ", count)
                # self.cancelmove()
                # self.move_Down2(y)
                self.level += 1

                if count%2 == 1:
                    self.cancelmove()
                    self.move_Right2(x)
                    self.cancelmove()

                    # self.lastY_Position= round((self.drone.get_state(AltitudeChanged)["altitude"]), 2)
                    # self.lastX_Position = round((self.drone.get_state(moveByEnd)["dY"]), 2)
                    self.lastY_Position= round((self.drone.get_state(AltitudeChanged)["altitude"]), 2)
                    self.lastX_Position = 1
                    print("Last Location X1: ", self.lastX_Position) # X is expected to be positive value
                    # print("Last Location Y1: ", self.lastY_Position)


                elif count%2 == 0:
                    self.cancelmove()
                    self.move_Left2(x)
                    self.cancelmove()

                    
                
                count = count-1
                if count != 0:
                    self.move_Up2(two_y)
                    
            else:
                print("---------------->>>> DONE SCANNING RACK")
                self.scanItemSignal = False
                self.changeRackSignal = True

        return
    
    def changeRack(self):
        
        if self.lastX_Position > 0:
            x = self.final_dX + 0.5
        
        elif self.lastX_Position < 0:
            x = 0
# ////// check balik sbb dia tak turun elok2 //////
        height = self.final_dY/self.actualRackLevel
        y = self.final_dY - height  ## height/2 
        self.changeRackSignal = True
        # self.move_Down2(-x,y)

        self.cancelmove()
        self.move_Right2(-x)
        self.cancelmove()
        self.move_Downnn(y)
        
        # print("Going down to scan changing rack tag")
        
        """Testing move down then land"""
        # self.land()

        """For Testing purposes"""
        return
        # self.cancelmove()
        # self.stopmove()
        # self.move_Left2(1)
        # self.cancelmove()
        # self.stopmove()
        # self.move_Forward(3)
        # self.cancelmove()
        # self.stopmove()
        # self.move_Right2(1)
        # self.cancelmove()
        # self.stopmove()
        # self.land()
        # self.changeRackSignal = False
        # self.scanItemSignal = True
        # print("latest change rack signal: ", self.changeRackSignal)
        # print("latest scan item signal: ", self.scanItemSignal)



    def scanItem(self):

        # if (self.dX_leftTop != 100) and (self.dX_rightTop != 100) and (self.dX_rightBottom != 100) and (self.dX_leftBottom != 100) and (self.dY_leftTop != 100) and (self.dY_leftTop != 100) and (self.dY_leftTop != 100) and (self.dY_leftTop != 100):
        self.X0 = self.dX_leftTop
        self.X1 = self.dX_rightTop
        self.Y0 = self.dY_leftBottom_afterTakeoff
        self.Y1 = self.dY_leftTop
        self.Y2 = self.dY_rightTop
        print(self.X0, " ", self.X1, " ", self.Y0, " ", self.Y1, " ", self.Y2)

        if (self.X0 != None) and (self.X1 != None) and (self.Y0 != None) and (self.Y1 != None) and (self.Y2 != None):

            print("******Enter scanning items operation******")
            print("******HORIZONTAL MOVEMENT******")
            
            # dX = round(dXX, 1)
            # dY = round(dYY, 1)
            self.rackNumber += 1
            dX = self.X1 +0.5
            dY = self.Y1 - self.Y0
            self.final_dX = round(dX, 2)
            self.final_dY = round(dY, 2)

            print("**----------> Final dX: ",self.final_dX)
            print("**----------> Final dY: ", self.final_dY)
           
            levelHeight = self.final_dY/self.actualRackLevel #will get height of each rack 
            dy_toMove = round( (levelHeight/2), 2) #lalu tengah2 rack.. 


            # upmovement = self.actualRackLevel
 
            # print("**----------> Movement drone should move up to: ", self.actualRackLevel) # based on actual rack level

            """ Go to first level """ 
            
            self.cancelmove()
            self.stopmove()
            # self.stopmove()
            self.move_Left2(self.final_dX)
            self.cancelmove()
            self.move_Downnn(self.final_dY - dy_toMove)

            self.scanningItemMovement(self.actualRackLevel, round(dy_toMove, 2), self.final_dX)

            # self.operationCalculationCounter = self.operationCalculationCounter + 1   
            # print("Operation Calculation Counter: ",self.operationCalculationCounter)
            # self.scanItemSignal = False
        return   



    def avoidCrash_usingDistance(self, distance):
        # print("Distance in CM:", distance)
        if distance >= 50.0 and distance <= 80.0:
            forwardBackward = 0
        elif distance > 80.0:
            forwardBackward = 0.5
        elif distance < 50.0:
            forwardBackward = -0.5


        return forwardBackward

    def storeData(self, barcodeData):
        # condition to check the barcode that have been scanned is an item ID or location ID
        # because of the library keep scanning and decode the frame, we need to set a condition if there is no QR code in the frame, just pass
        # if not self.barcodeData:
        #     pass
        # elif
        # print("barcode: " + barcodeData)
        if (barcodeData in self.listOfLocation):
            self.currentLocation = barcodeData
            self.currentLocationStatus = True
            print("*******Location status true detected******")
            print(self.barcodeDataList)

        elif (barcodeData not in self.barcodeDataList) and (self.currentLocationStatus == True):
            barcodeDataWithLevel = str(barcodeData) + " (level " + str(self.level) +")"
            self.barcodeDataList.append(barcodeData)
            # self.barcodeDataList.append(barcodeDataWithLevel)
            # item_dY = str(math.ceil((self.drone.get_state(AltitudeChanged)["altitude"])*100)/100)
            # item_dX = str(math.ceil((self.drone.get_state(moveByEnd)["dY"])*100)/100)
            # item_dX = str(self.drone.get_state(moveByEnd)["dY"])
            # self.itemlocation = "(" + item_dX + "," + item_dY + ")"

            # self.request_post.sendData(barcodeData, self.currentLocation, self.itemlocation)
            # self.request_post.sendData(barcodeData, self.currentLocation)
            self.request_post.sendData_CSV(barcodeDataWithLevel, self.currentLocation)
            print("*******Send data to sendData_CSV() in request post*******")


    def automove(self, movement):
        
        
        if movement == "UP":
            print("*******MOVE UP********")

            """ self.dY_leftBottom_afterTakeoff is Y0 """
            self.dY_leftBottom_afterTakeoff = round((self.drone.get_state(AltitudeChanged)["altitude"]), 1)

            print("**----------> Y0: ", self.dY_leftBottom_afterTakeoff)

            # stopmove() is to make sure the drone stop for a while before move to the next direction.. 
            # this is to avoid too much inertia
            self.stopmove()
            self.move_Up(self.forwardBackward, self.limitDistance)

     
        elif movement == "P3" or movement == "END":

            print("*********REACHING THIRD TAG OF THE RACK**********")
            
            self.cancelmove() # needed to get the distance X

            """
            dY_rightTop == Y2
            dX_rightTop == X1
            """
            
            self.dY_rightTop = round((self.drone.get_state(AltitudeChanged)["altitude"]), 1)
            self.dX_rightTop = round((self.drone.get_state(moveByEnd)["dY"]), 1)
            print("**----------> X1: ", self.dX_rightTop)
            print("**----------> Y2: ", self.dY_rightTop)

            self.stopmove()
            self.stopmove()
            self.cancelmove()
            # signalling ending the rack scanning
            self.scanRackSignal = False
            self.scanItemSignal = True
    

        elif movement == "RIGHT":
            print("**********MOVE RIGHT************")
            self.cancelmove() # needed to get the distance X
            
            """
            dY_leftTop == Y1
            dX_leftTop == X0
            """
            self.dY_leftTop = round((self.drone.get_state(AltitudeChanged)["altitude"]), 1)
            self.dX_leftTop = round((self.drone.get_state(moveByEnd)["dY"]), 1)
            print("**----------> Y1: ", self.dY_leftTop)
            print("**----------> X0: ", self.dX_leftTop)

            self.stopmove()
            self.move_Right(self.forwardBackward, self.limitDistance)

        elif movement == "LEFT":

            print("***********MOVE LEFT**************")
            self.cancelmove() # needed to get the distance X
            
            """
            dY_leftTop == Y1
            dX_leftTop == X0
            """
            self.dY_leftTop = round((self.drone.get_state(AltitudeChanged)["altitude"]), 1)
            self.dX_leftTop = round((self.drone.get_state(moveByEnd)["dY"]), 1)
            print("**----------> Y1: ", self.dY_leftTop)
            print("**----------> X0: ", self.dX_leftTop)

            self.stopmove()
            self.move_Left(self.forwardBackward, self.limitDistance)

        else:
            self.stopmove()
            print("*********[WARNING] No such direction [WARNING]**********")

        return
    

    def run(self):
        # print("run")
        window_name = "Olympe Streaming"
        # cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        # cv2.createButton('TakeOff', 	self.on_takeoff)
        # cv2.createButton('Land/Stop', 	self.on_land_stop)
        # cv2.createButton('Backoff and Land', 	self.on_backoff_land)
        main_thread = next(
            filter(lambda t: t.name == "MainThread", threading.enumerate())
        )
        
        # while self.drone.connect():
        while main_thread.is_alive():
            _, img = self.cap.read()
            self.event, self.values = self.window.read(timeout=20)
            
            if self.event == 'Exit' or self.event == sg.WIN_CLOSED:
                self.cap.release()
                return

            elif self.event == 'Start Streaming':
                self.recording = True
            
            elif self.event == 'End Streaming':
                self.recording = False
                # _, img = self.cap.read()
                img = np.full((480, 640), 255)
                # this is faster, shorter and needs less includes
                imgbytes = cv2.imencode('.png', img)[1].tobytes()
                self.window['image'].update(data=imgbytes)

            elif self.event == 'Start Operation':
                self.takeoff()

            elif self.event == 'End Operation':
                self.land()
                
            if self.cap.isOpened() and self.recording == True:
                # _, img = self.cap.read()
            
                try:
                    self.show_yuv_frame(window_name, img)
                    # imgbytes = cv2.imencode('.png', img)[1].tobytes()  # ditto
                    # self.window['image'].update(data=imgbytes)
                    # if (self.dX_2 != 100) and (self.dX_3 != 100) and (self.dX_4 != 100) and (self.dX_5 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100):
                    #     # self.scanItem()
                    #     print("hahaha")
                    #     # self.takeoff()
                    #     self.scanItem()
                    
                except Exception:
                    # We have to continue popping frame from the queue even if
                    # we fail to show one frame
                    traceback.print_exc()

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     anafi_connection.stop()
        # cv2.destroyWindow(window_name)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #         anafi_connection.stop()
            
        # if cv2.waitKey(1) & 0xFF == ord('l'):
        #     anafi_connection.land()

        # if cv2.waitKey(1) & 0xFF == ord('t'):
        #     anafi_connection.takeoff()

        # cv2.destroyWindow(window_name)
      

 
    """""""""""""""""""""""""""""""""""""""DRONE MOVEMENT CODE"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
    def takeoff(self):
        # assert     
        self.drone(
        TakeOff()
        >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()
        print("---------------------------------------------------SUCCESSFUL TAKEOFF---------------------------------------------------------------------")

        return
    
    def land(self):
        self.cancelmove()
        #Move backward before landing... just for safety now
        #extended_move_by(front, right, down, rotation, max_horizontal_speed, max_vertical_speed, max_yaw_rotation_speed)
        # self.drone(extended_move_by(-1.0, 0, 0, 0, 1, 1, 0)).wait().success()
        # assert 
        self.drone(Landing()).wait().success()
        print("---------------------------------------------------SUCCESSFUL LAND---------------------------------------------------------------------")

    def cancelmove(self):
        self.drone(CancelMoveBy()).wait().success()
        self.forwardBackward = 0
        print("---------------------------------------------------SUCCESSFUL CANCEL---------------------------------------------------------------------")

        return

    def stopmove(self):
        self.drone(extended_move_by(0, 0, 0, 0, 0, 0, 0)).wait(1).success()
        print("---------------------------------------------------SUCCESSFUL REST---------------------------------------------------------------------")
       
        return

    def move_Forward(self, range):
        distance = float(range)
        # assert 
        self.drone(
        moveBy(distance, 0, 0, 0)
        # >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()
        print("---------------------------------------------SUCCESSFUL FORWARD---------------------------------------------------------------------")

        return
    
    def move_Backward(self, range):
        distance = -float(range)
        # assert 
        self.drone(
        extended_move_by(distance, 0, 0, 0, 1, 1, 0)
        # >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()
        print("---------------------------------------------SUCCESSFUL BACKWARD---------------------------------------------------------------------")

        return
    
    def move_Right(self, forwardBackward, range):
        distance = float(range)
        #extended_move_by(d_x, d_y, d_z, d_psi, max_horizontal_speed, max_vertical_speed, max_yaw_rotation_speed, _timeout=10, _no_expect=False, _float_tol=(1e-07, 1e-09))
        self.drone(extended_move_by(forwardBackward, 0, 0, 0, 1, 1, 0)).wait().success()
        self.drone(extended_move_by(0, distance, 0, 0, 1, 1, 0))
            # >> FlyingStateChanged(state="hovering", _timeout=5)
        # )
        # print("---------------------------------------------------SUCCESSFUL RIGHT---------------------------------------------------------------------")
        return

    def move_Right2(self, range):
        distance = float(range)
        """extended_move_by(d_x, d_y, d_z, d_psi, max_horizontal_speed, max_vertical_speed, max_yaw_rotation_speed, _timeout=10, _no_expect=False, _float_tol=(1e-07, 1e-09))"""
        self.drone(extended_move_by(self.forwardBackward, distance, 0, 0, 1.0, 1.5, 0)).wait().success()

        print("---------------------------------------------------SUCCESSFUL RIGHT2---------------------------------------------------------------------")
        return

    def move_Left2(self, range):
        distance = -float(range)
        self.drone(extended_move_by(0, distance, 0, 0, 1.5, 1.5, 0)).wait().success()
        print("---------------------------------------------------SUCCESSFUL LEFT2---------------------------------------------------------------------")
        return

    def move_Up2(self, range):
        distance = -float(range)
        self.drone(extended_move_by(self.forwardBackward, 0, distance, 0, 1.5, 1.5, 0)).wait().success()
            # >> FlyingStateChanged(state="hovering", _timeout=10)
        # )
        print("---------------------------------------------------SUCCESSFUL UP2---------------------------------------------------------------------")
        return

    def move_Down2(self, Xrange, Yrange):
        distance = float(Yrange)
        self.drone(extended_move_by(self.forwardBackward, Xrange, distance, 0, 1, 1, 0)).wait().success()
            # >> FlyingStateChanged(state="hovering", _timeout=10)
        # )
        print("---------------------------------------------------SUCCESSFUL DOWN2---------------------------------------------------------------------")
        return


    def move_Downnn(self, Yrange):
        distance = float(Yrange)
        self.drone(extended_move_by(self.forwardBackward, 0, distance, 0, 1, 1, 0)).wait().success()
            # >> FlyingStateChanged(state="hovering", _timeout=10)
        # )
        print("---------------------------------------------------SUCCESSFUL DOWN2---------------------------------------------------------------------")
        return

    def move_Left(self, forwardBackward, range):
        distance = -float(range)
        self.drone(extended_move_by(forwardBackward, 0, 0, 0, 1, 1, 0)).wait().success()
        self.drone(extended_move_by(0, distance, 0, 0, 1, 1, 0))
            # >> FlyingStateChanged(state="hovering", _timeout=5)
        # )
        # print("---------------------------------------------------SUCCESSFUL LEFT---------------------------------------------------------------------")
        return
    
    def move_Up(self, forwardBackward, range):
        distance = -float(range)
        self.drone(extended_move_by(forwardBackward, 0, 0, 0, 1, 1, 0)).wait().success()
        self.drone(extended_move_by(0, 0, distance, 0, 1, 1, 0))
            # >> FlyingStateChanged(state="hovering", _timeout=10)
        # )
        # print("---------------------------------------------------SUCCESSFUL UP---------------------------------------------------------------------")
        return

    def move_Down(self, forwardBackward, range):
        distance = float(range)
        self.drone(extended_move_by(forwardBackward, 0, 0, 0, 1, 1, 0)).wait().success()
        self.drone(extended_move_by(0, 0, distance, 0, 1, 1, 0))
            # >> FlyingStateChanged(state="hovering", _timeout=10)
        # )ss
        # print("---------------------------------------------------SUCCESSFUL DOWN---------------------------------------------------------------------")
        return

    def changeRack2(self, range1, range2, range3):
        # range1 = range1 + 0.4

        print("....>>>>>> moving to next rack")
        self.cancelmove()
        self.stopmove()
        self.move_Left2(range1)
        self.cancelmove()
        self.stopmove()
        self.move_Forward(float(range2))
        self.cancelmove()
        self.stopmove()
        self.move_Right2(range3)
        self.cancelmove()
        self.stopmove()
        self.changeRackSignal = False
        self.scanRackSignal = True

        return
        # self.land()


    def fullyAutonomous(self):

        takeoffStatus = self.drone.get_state(FlyingStateChanged)["state"]

        if takeoffStatus != FlyingStateChanged_State.landed :
            print(takeoffStatus)

        """Test Changing Rack"""
        # self.takeoff()      
        # # self.land()
        # self.move_Up2(2)
        # self.cancelmove()
        # self.stopmove()
        # self.move_Right2(2)
        # self.scanRackSignal = False
        # self.scanItemSignal = True
        # # # print("signal 1: ", )
        # self.rackNumber = 1
        # x = 2
        # goToFirstLevel_Y = 2 - 0.5
        # self.takeoff()
        # self.move_Up2(2)
        # self.cancelmove()
        # self.move_Right2(2)
        # self.cancelmove()
        # self.move_Down2(-x, goToFirstLevel_Y)
        # self.scanRackSignal = False
        # self.scanItemSignal = True
        """"""
        try:
                
            while self.drone_in_operation == True:
                
                if self.drone.get_state(FlyingStateChanged)["state"] != FlyingStateChanged_State.landed:
                    
                    # if (self.forceLandingSignal == True):
                    #     print("breaking")
                    #     break

                    # if (self.operationCalculationCounter == 2):
                    #     self.land()

                    """Activating scanning tags on items/packages"""
                    if (self.scanRackSignal == False) and (self.scanItemSignal == True) and (self.changeRackSignal == False):
                        # print("Activating scanning tags on items/package")
                        # self.land() #use this when testing kalau taknak masuk scanning item after scan rack tags
                        self.scanItem() # will use this one for real operation
                        # print("Entering rack Scanning")

                    """Activating scanning tags on rack"""
                    if (self.scanRackSignal == True) and (self.scanItemSignal == False) and (self.currentInstruction in self.rackInstructions):
                            if self.inst != self.currentInstruction:
                                self.inst = self.currentInstruction
                                # self.automove(self.currentInstruction, self.forwardBackward)
                                print("[IN FULLY AUTONOMOUS]----------> Current Intruction: ", self.currentInstruction)
                                self.automove(self.currentInstruction)
                                # print("entering automove")
                                # inst = self.currentInstruction
                            else:
                                pass

                    """Activating scanning tags to change rack"""
                    if (self.scanItemSignal == False) and (self.scanRackSignal == False) and (self.changeRackSignal == True):
                        
                        if self.cr == 0:
                            self.changeRack()
                            self.cr = self.cr + 1
                        # try:
                        #     print("------>>>>start changing rack")
                        
                        #     print("------>>>>end changing rack")
                        # except:
                        #     print("stopppp")

                        if self.cr1 != None :
                            self.changeRack2(self.cr1, self.cr2, self.cr3)

                        else:
                            pass
                    else:
                        pass
                
                else:
                    pass

            # while self.drone_in_operation == 1 :
            #     pass
        
        except:
            print("Something went wrong")
            self.land()



if __name__ == "__main__":
    anafi_connection = AnafiConnection()
    anafi_connection.start()
    anafi_connection.fullyAutonomous()
    anafi_connection.stop()

       
        



