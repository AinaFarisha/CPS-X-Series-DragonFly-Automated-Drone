#!/usr/bin/env python

# NOTE: Line numbers of this example are referenced in the user guide.
# Don't forget to update the user guide after every modification of this example.
# source ~/code/parrot-groundsdk/./products/olympe/linux/env/shell


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

DRONE_IP = "192.168.42.1"
RACK_LEVEL = 3

class AnafiConnection(threading.Thread):

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(
            DRONE_IP, drone_type=od.ARSDK_DEVICE_TYPE_ANAFI4K)
       

        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")
        print("Olympe streaming output dir: {}".format(self.tempd))
        self.h264_frame_stats = []
        self.h264_stats_file = open(os.path.join(self.tempd, 'h264_stats.csv'), 'w+')
        self.h264_stats_writer = csv.DictWriter(self.h264_stats_file, ['fps', 'bitrate'])
        self.h264_stats_writer.writeheader()
        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()

        """For gui"""
        self.recording = False

        sg.theme('Black')
        self.layout = [
            #   [sg.Text('OpenCV Demo', size=(40, 1), justification='center', font='Helvetica 20')],
              [sg.Image(filename='', key='image'),],
              [sg.Button('Start Streaming', size=(10, 1), font='Helvetica 12'),
               sg.Button('End Streaming', size=(10, 1), font='Helvetica 12'),
               sg.Button('Start Operation', size=(10, 1), font='Helvetica 12'),
               sg.Button('End Operation', size=(10, 1), font='Helvetica 12'),
               sg.Button('Exit', size=(10, 1), font='Helvetica 12'), ],
               [sg.Text('Zoom Level: ', size=(10, 1), justification='center',font='Helvetica 12'), sg.Slider(range=(1.0,3.0), key='zoomlevel', default_value=1.0, resolution=.1, size=(50,15), orientation='horizontal')]
               ]

        
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
        # Intructions DOWN should not be one of the instruction
        self.rackInstructions = ["UP", "RIGHT", "LEFT", "END"]
        self.inst = None

        # chnage rack
        self.cr = 0
        
        # rack related
        self.rackNumber = 0
        self.level = 0
        self.actualRackLevel = RACK_LEVEL 

        # initializing the barcode data list for storing the list of barcode that will be scanned later
        self.barcodeDataList = []

        # initializing for forward and backward movement to avoid crash
        self.forwardBackward = 0
        
        # current scanned barcode data
        self.barcodeData = ""

        """Distance Related"""
        #for distance x and distance y for scanning area
        self.final_dX = 0.0
        self.final_dY = 0.0
        
        # to check the location after scanning all items
        self.lastX_Position = self.lastY_Position = 0

        # default position before scanning rack
        self.dY_leftBottom_afterTakeoff = self.dX_left_Right_Top = self.dY_left_Right_Top = self.dX_rightTop = self.dY_rightTop = self.dX_rightBottom = self.dY_rightBottom = self.dX_leftBottom = self.dY_leftBottom = None
        
        # maximum distance for drone to move 
        self.limitDistance = 10
        
        self.warningSignal = None
        # self.forceLandingSignal = False
        """distance for calculation"""
        self.X0 = None
        self.X1 = None
        self.Y0 = None
        self.Y1 = None
        self.Y2 = None


        """change rack instrcution"""
        self.cr1 = None
        self.cr2 = None
        self.cr3 = None
        
        """flag signal"""
        self.drone_in_operation = True
        self.scanRackSignal = True
        self.scanItemSignal = False
        self.changeRackSignal = False
        self.zoomLevel = 1.0

        self.point_2 = None

        """GUI"""
        self.event = None
        self.values = None

        """"""
        # FPS = 1/X
        # X = desired FPS
        self.FPS = 1/30
        self.FPS_MS = int(self.FPS * 1000)

        super().__init__()
        # print("Initialization succesfull, Drone is ready to FLY")
        super().start()

    # connect olympe with the drone and setup callback functions for the olympe SDK
    def start(self):
        # Connect the the drone)
        try:
            
            self.drone.connect()
            if self.drone.connect():
                print("**----------> Drone Connected")
                print("**----------> Battery percentage before takeoff:", self.drone.get_state(BatteryStateChanged)["percent"])
                print("**----------> Altitude before takeoff(should be 0):", self.drone.get_state(AltitudeChanged)["altitude"])

                # Setup your callback functions to do some live video processing
                self.drone.set_streaming_callbacks(
                    raw_cb=self.yuv_frame_cb,
                    h264_cb=self.h264_frame_cb,
                    start_cb=self.start_cb,
                    end_cb=self.end_cb,
                    flush_raw_cb=self.flush_cb,
                )

             
                self.drone.start_video_streaming()
                
            else:
                print("**---------->[ERROR] Drone NOT CONNECTED [ERROR]<----------**")
        except:
            print("Error at starting")

        print("Initialization succesfull, Drone is ready to FLY")


    # Properly stop the video stream and disconnect
    def stop(self):
        self.drone.stop_video_streaming()
        self.drone.disconnect()
        self.h264_stats_file.close()
        

    # This function will be called by Olympe for each decoded YUV frame.
    def yuv_frame_cb(self, yuv_frame):
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)

    # This function will be called by Olympe to flush the callback
    def flush_cb(self):
        with self.flush_queue_lock:
            while not self.frame_queue.empty():
                self.frame_queue.get_nowait().unref()
        return True

    # This function is necessary for Olympe SDK
    def start_cb(self):
        pass

    # This function is necessary for Olympe SDK
    def end_cb(self):
        pass

    # This function will be called by Olympe for each new h264 frame.
    def h264_frame_cb(self, h264_frame):

        # Get a ctypes pointer and size for this h264 frame
        frame_pointer, frame_size = h264_frame.as_ctypes_pointer()

        # Compute some stats and dump them in a csv file
        info = h264_frame.info()
        frame_ts = info["ntp_raw_timestamp"]
        if not bool(info["h264"]["is_sync"]):
            if len(self.h264_frame_stats) > 0:
                while True:
                    start_ts, _ = self.h264_frame_stats[0]
                    if (start_ts + 1e6) < frame_ts:
                        self.h264_frame_stats.pop(0)
                    else:
                        break
            self.h264_frame_stats.append((frame_ts, frame_size))
            h264_fps = len(self.h264_frame_stats)
            h264_bitrate = (
                8 * sum(map(lambda t: t[1], self.h264_frame_stats)))
            self.h264_stats_writer.writerow(
                {'fps': h264_fps, 'bitrate': h264_bitrate})

    def show_yuv_frame(self, window_name, yuv_frame):
        # the VideoFrame.info() dictionary contains some useful information
        # such as the video resolution
        info = yuv_frame.info()
        height, width = info["yuv"]["height"], info["yuv"]["width"]
        
        # print(height, width)

        # yuv_frame.vmeta() returns a dictionary that contains additional
        # metadata from the drone (GPS coordinates, battery percentage, ...)

        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]

        # yuv_frame.as_ndarray() is a 2D numpy array with the proper "shape"
        # i.e (3 * height / 2, width) because it's a YUV I420 or NV12 frame

        # Use OpenCV to convert the yuv frame to RGB
        cv2frame = cv2.cvtColor(
            yuv_frame.as_ndarray(), cv2_cvt_color_flag)

        dsize = (640,450) #frame size

        #if the live streaming is not displayed. try to uncommet and run, then comment and run the code again
        cv2frame = cv2.resize(cv2frame, dsize) #frame size 

        
        # camera zoom update
        self.zoomLevel = self.values['zoomlevel']
        self.drone(
            set_zoom_target(
                cam_id=0,
                control_mode=0,
                target=self.zoomLevel
                )
            ) 

        zoomlevel = str(self.zoomLevel) + "x" 
        # shows zoom level on screen 
        cv2.putText(cv2frame, zoomlevel, (cv2frame.shape[1] - 60, cv2frame.shape[0] - 430), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # shows battery percentage on camera screen
        batteryPercentage = self.drone.get_state(BatteryStateChanged)["percent"]
        batteryText = "Battery Level: " + str(batteryPercentage) + "%"
        if (batteryPercentage < 25):
            self.warningSignal = 3
            cv2.putText(cv2frame, batteryText, (cv2frame.shape[1] - 620, cv2frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        else:
            cv2.putText(cv2frame, batteryText, (cv2frame.shape[1] - 620, cv2frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if (self.warningSignal != None) :

            if (self.warningSignal == 1):
                warningText = "WARNING!!! FORCE LANDING"
                cv2.putText(cv2frame, warningText, (cv2frame.shape[1] - 1250, cv2frame.shape[0] - 650), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 255), 3)
            
            elif (self.warningSignal == 2):
                warningText = "WARNING!!! BATTERY LOW"
                cv2.putText(cv2frame, warningText, (cv2frame.shape[1] - 1250, cv2frame.shape[0] - 650), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 255), 3)
            
            else:
                pass

        
        """
        # scan the barcode, draw box and data in the frame
        # returning barcode and distance in cm 
        # barcodeInfo[0] == barcode and barcodeInfo[1] == distance 
        """
        self.barcodeInfo  = self.scanning_decode.startScanning(cv2frame, self.focalLength, self.KNOWN_WIDTH)
        

        if not self.barcodeInfo:
            pass

        else:
 
            distance = self.barcodeInfo[1]
            self.forwardBackward = self.avoidCrash_usingDistance(float(distance)) #//boleh amik nnt kalau nak guna 
  
            
            self.barcodeData1 = str(self.barcodeInfo[0])
            # print(self.barcodeData)

            # to read the tags only once at a time 
            if self.barcodeData1 != self.barcodeData:
                self.barcodeData = str(self.barcodeInfo[0])
                codeInfo= str(self.barcodeData).split()
                print("codeInfo: ", codeInfo[0])

                if len(codeInfo) > 1: 

                    if (self.changeRackSignal == True) and (codeInfo[0] == "CR") :
                        self.cr1 = float(codeInfo[1])
                        self.cr2 = float(codeInfo[2])
                        self.cr3 = float(codeInfo[3])
                        
                        # print("latest change rack signal: ", self.changeRackSignal)
                        # print("latest scan item signal: ", self.scanItemSignal)
                        
                    else:
                        self.storeData(codeInfo[0]) # store rack data
                        if (codeInfo[1] == "LAND" and self.changeRackSignal == True):
                            self.land()
                        else:
                            self.currentInstruction = codeInfo[1]
                            print(">>>>>current movement instruction: ", self.currentInstruction)
                            print(">>>>>distance between code and drone: ", distance)
                            print(">>>>>forwardBackward collision: ", self.forwardBackward)

                else:
                    self.storeData(codeInfo[0])
                    print(">>>>>distance between code and drone: ", distance)
                    print(">>>>>forwardBackward collision: ", self.forwardBackward)

        # for PySimpleGUI
        imgbytes = cv2.imencode('.png', cv2frame)[1].tobytes()
        self.window['image'].Update(data=imgbytes)
        # Use OpenCV to show this frame
        # cv2.imshow(window_name, cv2frame)
        # cv2.waitKey(self.FPS_MS)  # please OpenCV for 1 ms...

    def scanningItemMovement(self, count, y, x):
        count2 = count
        two_y = y*2
        print("**----------> distance x that drone should travel: ", x)
        print("**----------> distance y that drone shoud travel: ", y)
        self.level = 0

        if self.point_2 == "RIGHT" :
            pass

        elif self.point_2 == "LEFT":
            x = -(x)

        else:
            pass

        while count > 0:
            self.level += 1
            self.cancelmove()
            self.moveRightLeft(True, self.forwardBackward, x)
            self.cancelmove()
            

            if count == 1 :
                self.lastX_Position = x
                print("Last Location X1: ", self.lastX_Position)

            else:
                self.moveUp(True, self.forwardBackward, two_y)
                pass
            
            
            x = -(x)
            count -= 1

        else:
            print("---------------->>>> DONE SCANNING RACK")
            #end scanning item and begin change rack
            self.scanItemSignal = False
            self.changeRackSignal = True

        return

    
    def changeRack(self):
        
        if (self.lastX_Position > 0 and self.point_2 == "RIGHT"):
            x = -(self.final_dX)
        
        elif (self.lastX_Position < 0 and self.point_2 == "LEFT"):
            x = (self.final_dX) 

        elif (self.lastX_Position < 0 and self.point_2 == "RIGHT") or (self.lastX_Position > 0 and self.point_2 == "LEFT"):
            x = 0

        else:
            pass


        level_height = self.final_dY/self.actualRackLevel
        height = level_height/2
        y = self.final_dY - height  ## height/2 
  

        self.cancelmove()

        self.moveRightLeft(True, self.forwardBackward, x)
        self.cancelmove()
  
        self.moveDown(True, self.forwardBackward, y)
 
        return

    
    def moveToNextRack(self, range1, range2, range3):
    

        print("....>>>>>> moving to next rack")
        # print(self.point_2)

        if (self.point_2 == "RIGHT"):
            range1 = -(range1)

        elif (self.point_2 == "LEFT"):
            range3 = -(range3)
        
        else:
            print("NOT LEFT RIGHT")

        print(range1," " ,range2, " ",range3)

        self.cancelmove()
    
        self.moveRightLeft(True, self.forwardBackward, range1)
        self.cancelmove()

        self.move_Forward(float(range2))
        self.cancelmove()

        self.moveRightLeft(True, self.forwardBackward, range3)

        self.cancelmove()
        self.changeRackSignal = False
        self.scanRackSignal = True
        self.cr = 0

        return


    def scanItem(self):

        
        self.X0 = self.dX_left_Right_Top
        self.X1 = abs(self.dX_rightTop) # x after reach end
        self.Y0 = self.dY_leftBottom_afterTakeoff
        self.Y1 = self.dY_left_Right_Top
        self.Y2 = self.dY_rightTop
        print(self.X0, " ", self.X1, " ", self.Y0, " ", self.Y1, " ", self.Y2)

        if (self.X0 != None) and (self.X1 != None) and (self.Y0 != None) and (self.Y1 != None) and (self.Y2 != None):

            print("******Enter scanning items operation******")
            print("******HORIZONTAL MOVEMENT******")

            # dX = round(dXX, 1)
            # dY = round(dYY, 1)
            self.rackNumber += 1
            dX = self.X1 + 0.3
            dY = self.Y1 - self.Y0
            self.final_dX = round(dX, 2)
            self.final_dY = round(dY, 2)

            print("**----------> Final dX: ",self.final_dX)
            print("**----------> Final dY: ", self.final_dY)
           
            levelHeight = self.final_dY/self.actualRackLevel #will get height of each rack 
            dy_toMove = round( (levelHeight/2), 2) #half level


            """ Go to middle of first level """ 
            if (self.point_2 == "RIGHT"):
                dX = -(self.final_dX)
                print(dX)

            elif (self.point_2 =="LEFT"):
                dX = self.final_dX
                print(dX)

            self.moveRightLeft(True, self.forwardBackward, dX)
            self.cancelmove()
            movedistance = round((self.drone.get_state(moveByEnd)["dY"]), 2)
            # print("move distance : ", movedistance)
            # self.move_Downnn(self.final_dY - dy_toMove)
            # print("move down: ", (self.final_dY - dy_toMove))

            self.moveDown(True, self.forwardBackward, (self.final_dY - dy_toMove))
            

            self.scanningItemMovement(self.actualRackLevel, round(dy_toMove, 2), self.final_dX)

        return   



    def avoidCrash_usingDistance(self, distance):
        # print("Distance in CM:", distance)
        if distance >= 50.0 and distance <= 90.0:
            forwardBackward = 0
        elif distance > 90.0:
            forwardBackward = 0.2
        elif distance < 50.0:
            forwardBackward = -0.2

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
     
            self.moveUp(False, self.forwardBackward, self.limitDistance)

     
        elif movement == "END":

            print("*********REACHING THIRD TAG OF THE RACK**********")
            
            self.cancelmove() # needed to get the distance X

            """
            dY_rightTop == Y2
            dX_rightTop == X1
            """
            
            self.dY_rightTop = round((self.drone.get_state(AltitudeChanged)["altitude"]), 2)
            self.dX_rightTop = round((self.drone.get_state(moveByEnd)["dY"]), 2)
            print("**----------> X1: ", self.dX_rightTop)
            print("**----------> Y2: ", self.dY_rightTop)

            self.stopmove()
            # self.cancelmove()
            # signalling ending the rack scanning
          
            self.scanRackSignal = False
            self.scanItemSignal = True


        elif movement == "RIGHT":
            print("**********MOVE RIGHT************")
            self.cancelmove() # needed to get the distance X
            
            """
            dY_left_Right_Top == Y1
            dX_leftTop == X0
            """
            self.dY_left_Right_Top = round((self.drone.get_state(AltitudeChanged)["altitude"]), 1)
            self.dX_left_Right_Top = round((self.drone.get_state(moveByEnd)["dY"]), 1)
            print("**----------> Y1: ", self.dY_left_Right_Top)
            print("**----------> X0: ", self.dX_left_Right_Top)

   
            self.point_2 = movement
            self.moveRightLeft(False, self.forwardBackward, self.limitDistance)

        elif movement == "LEFT":

            print("***********MOVE LEFT**************")
            self.cancelmove() # needed to get the distance X
            
            """
            dY_left_Right_Top == Y1
            dX_leftTop == X0
            """
            self.dY_left_Right_Top = round((self.drone.get_state(AltitudeChanged)["altitude"]), 1)
            self.dX_left_Right_Top = round((self.drone.get_state(moveByEnd)["dY"]), 1)
            print("**----------> Y1: ", self.dY_left_Right_Top)
            print("**----------> X0: ", self.dX_left_Right_Top)

            self.stopmove()
            self.cancelmove()

            self.point_2 = movement
            self.moveRightLeft(False, self.forwardBackward, -(self.limitDistance))

        else:
            self.stopmove()
            print("*********[WARNING] No such direction [WARNING]**********")

        return
    

    
    # This function is necessary for the Olympe SDK. It will be called by Olympe
    def run(self):
        window_name = "Olympe Parrot Camera Live Streaming"
  
        main_thread = next(
            filter(lambda t: t.name == "MainThread", threading.enumerate())
        )
        
        while main_thread.is_alive():

            self.event, self.values = self.window.read(timeout=20)
                
            if self.event == 'Exit' or self.event == sg.WIN_CLOSED:
                # self.cap.release()
                self.stop()
                return

            elif self.event == 'Start Streaming':
                self.recording = True
            
            elif self.event == 'End Streaming':
                self.recording = False
                # _, img = self.cap.read()
                # yuv_frame = np.full((480, 640), 255)
                # # this is faster, shorter and needs less includes
                # imgbytes = cv2.imencode('.png', yuv_frame)[1].tobytes()
                # self.window['image'].update(data=imgbytes)

            elif self.event == 'Start Operation':
                self.takeoff()

            elif self.event == 'End Operation':
                self.land()


            with self.flush_queue_lock:
                try:
                    yuv_frame = self.frame_queue.get(timeout=0.01)
                except queue.Empty:
                    continue
                
                if self.recording:

                    try:
                        self.show_yuv_frame(window_name, yuv_frame)
                        
                    except Exception:
                        # We have to continue popping frame from the queue even if
                        # we fail to show one frame
                        traceback.print_exc()
                        
                    finally:
                        # Don't forget to unref the yuv frame. We don't want to
                        # starve the video buffer pool
                        yuv_frame.unref()
                
                       
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
        self.drone(Landing()).wait().success()
        print("---------------------------------------------------SUCCESSFUL LAND---------------------------------------------------------------------")
    
    def cancelmove(self):
        self.drone(CancelMoveBy()).wait().success()
        self.forwardBackward = 0
        # print("---------------------------------------------------SUCCESSFUL CANCEL---------------------------------------------------------------------")

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
        # print("---------------------------------------------SUCCESSFUL FORWARD---------------------------------------------------------------------")

        return
    
    # def move_Backward(self, range):
    #     distance = -float(range)
    #     # assert 
    #     self.drone(
    #     extended_move_by(distance, 0, 0, 0, 0.2, 0.2, 0.2)
    #     # >> FlyingStateChanged(state="hovering", _timeout=5)
    #     ).wait().success()
    #     print("---------------------------------------------SUCCESSFUL BACKWARD---------------------------------------------------------------------")

    #     return
    
    # def moveRight(self, waitSuccess, forwardBackward, range):
    #     distance = float(range)
    #     """extended_move_by(d_x, d_y, d_z, d_psi, max_horizontal_speed, max_vertical_speed, max_yaw_rotation_speed, _timeout=10, _no_expect=False, _float_tol=(1e-07, 1e-09))"""
    #     # self.stopmove()
    #     self.drone(extended_move_by(0, 0, 0, 0, 0, 0, 0)).wait(5).success()
    #     self.drone(extended_move_by(forwardBackward, 0, 0, 0, 1, 1, 0)).wait().success()

    #     if (waitSuccess == True):
    #         # self.drone(extended_move_by(0, distance, 0, 0, 1, 1, 0)).wait().success()
    #         self.drone(moveBy(0, distance, 0, 0)).wait().success()

    #     else:
    #         self.drone(extended_move_by(0, distance, 0, 0, 0.5, 0.5, 0))

    #     return


    # def moveLeft(self, waitSuccess, forwardBackward, range):
    #     distance = -float(range)
    #     self.stopmove()
    #     self.drone(extended_move_by(forwardBackward, 0, 0, 0, 1, 1, 0)).wait().success()

    #     if (waitSuccess == True):
    #         self.drone(extended_move_by(0, distance, 0, 0, 1, 1, 0)).wait().success()

    #     else:
    #         self.drone(extended_move_by(0, distance, 0, 0, 1, 1, 0))

    #     return


    def moveRightLeft(self, waitSuccess, forwardBackward, range):
        print("---------------------------------------------MOVING RIGHT OR LEFT---------------------------------------------------------------------")
        distance = float(range)
        self.stopmove()
        self.drone(extended_move_by(forwardBackward, 0, 0, 0, 1, 1, 0)).wait().success()

        if (waitSuccess == True):
            self.drone(extended_move_by(0, distance, 0, 0, 1, 1, 0)).wait().success()

        else:
            self.drone(extended_move_by(0, distance, 0, 0, 1, 1, 0))

        return


    def moveUp(self, waitSuccess, forwardBackward, range):
        print("---------------------------------------------MOVING UP---------------------------------------------------------------------")
        distance = -float(range)
        self.stopmove()
        self.drone(extended_move_by(forwardBackward, 0, 0, 0, 1, 1, 0)).wait().success()

        if (waitSuccess == True):
           self.drone(extended_move_by(0, 0, distance, 0, 1, 1, 0)).wait().success()

        else:
            self.drone(extended_move_by(0, 0, distance, 0, 1, 1, 0))


    def moveDown(self, waitSuccess, forwardBackward, range):
        print("---------------------------------------------MOVING UP---------------------------------------------------------------------")
        distance = float(range)
        self.drone(extended_move_by(forwardBackward, 0, 0, 0, 1, 1, 0)).wait().success()

        if (waitSuccess == True):
            self.drone(extended_move_by(0, 0, distance, 0, 1, 1, 0)).wait().success()

        else:
            self.drone(extended_move_by(0, 0, distance, 0, 1, 1, 0))     


    def fullyAutonomous(self):


        takeoffStatus = self.drone.get_state(FlyingStateChanged)["state"]

        if takeoffStatus != FlyingStateChanged_State.landed :
            print(takeoffStatus)

                
        try:
                
            while self.drone_in_operation == True:
                
                if self.drone.get_state(FlyingStateChanged)["state"] != FlyingStateChanged_State.landed:
                    

                    """Activating scanning tags on items/packages"""
                    if (self.scanRackSignal == False) and (self.scanItemSignal == True) and (self.changeRackSignal == False):
                        # print("Activating scanning tags on items/package")     
                        self.scanItem() # will use this one for real operation


                    """Activating scanning tags on rack"""
                    if (self.scanRackSignal == True) and (self.scanItemSignal == False) and (self.currentInstruction in self.rackInstructions):
                            if self.inst != self.currentInstruction:
                                self.inst = self.currentInstruction
                                # self.automove(self.currentInstruction, self.forwardBackward)
                                print("[IN FULLY AUTONOMOUS]----------> Current Intruction: ", self.currentInstruction)
                                self.automove(self.currentInstruction)
    
                            else:
                                pass

                    """Activating scanning tags to change rack"""
                    if (self.scanItemSignal == False) and (self.scanRackSignal == False) and (self.changeRackSignal == True):
                        
                        if self.cr == 0:
                            self.changeRack()
                            self.cr = self.cr + 1

                        elif self.cr1 != None :
                            self.moveToNextRack(self.cr1, self.cr2, self.cr3)

                        else:
                            pass
                    else:
                        pass
                
                else:
                    pass

        
        except Exception as e:
            print("xxxxxxxxxx>>>>>> ERROR: ", e)
            self.land()


# Main function
if __name__ == "__main__":
    anafi_connection = AnafiConnection()
    anafi_connection.start()
    anafi_connection.fullyAutonomous()
    anafi_connection.stop()
