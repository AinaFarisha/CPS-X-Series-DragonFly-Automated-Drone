import requests
import argparse
import cv2
from pyzbar import pyzbar
import numpy as np
import imutils
import datetime
import time
from csv import writer
# import csv


class Anafi_Request_Post():

    found = set()
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--output", type=str, default="barcodes.csv", help="path to output CSV file containing barcodes")
    args = vars(ap.parse_args())
    csv = open(args["output"], "w")

    # read the location from the txt file
    # and return the location back in list
    def readLocation(self):
        locationList = []
        # change the location file path according to your path
        # locationFile = '/home/dragonfly/UsingController/listOfLocation.txt'
        locationFile = 'listOfLocation.txt'

        file = open(locationFile, "r")
        locations = file.readlines()

        # loop over the location and append it to the list
        for location in locations:
            if location != '' and location != '\n':
                location = location.rstrip()
                locationList.append(location)
        return locationList
    
    # def sendData(self, idData, locationData, itemLocation):
    def sendData(self, idData, locationData):
        itemfile = "itemsWithLocation.txt"
        file = open(itemfile, "a")
        # file.write(locationData + " " + idData + " " + itemLocation + "\n")
        file.write(locationData + " " + idData + "\n")
        # print("SENT -->  " + locationData + " " + idData + " " + itemLocation)
        print("SENT -->  " + locationData + " " + idData)

        file = open(itemfile, "r")
        self.write_csv_file(idData)
        
        # ap = argparse.ArgumentParser()
        # ap.add_argument("-o", "--output", type=str, default="barcodes.csv", help="path to output CSV file containing barcodes")
        # args = vars(ap.parse_args())
        
        # """csv"""
        # csv = open(args["output"], "w")
        # found = set()
        # write_csv_file(idData)

    def write_csv_file(self, data):
        if data not in Anafi_Request_Post.found:
            Anafi_Request_Post.csv.write("{},{}\n".format(datetime.datetime.now(), data))
            Anafi_Request_Post.csv.flush()
            Anafi_Request_Post.found.add(data)

    def sendData_CSV(self, data, locationData):

        if data not in Anafi_Request_Post.found:
            Anafi_Request_Post.csv.write("{},{},{}\n".format(datetime.datetime.now(), data, locationData))
            Anafi_Request_Post.csv.flush()
            Anafi_Request_Post.found.add(data)
            print("SENT -->  " + locationData + " " + data)
            






    # get the list of location from the server
    # and return back the location in list
    def getLocation(self):
        locationList = []
        url = 'https://cpsdragonfly.herokuapp.com/api/location'
        auth = {'Authorization': '$2y$10$.wGNf1JYzHOjRF5xLFovSuEeJGk9XQ4hPaIGvY0D6jpaHR4Ib5/tO'}
        request_get = requests.get(url, headers=auth)

        # load the location in json
        loadLocation = request_get.json()

        # loop over the loaded location and append it to the list,
        # then return the location list in list
        for i in range(len(loadLocation)):
            tempLocation = loadLocation[i]
            locationList.append(tempLocation['location_description'])
        return locationList

    # send the idData and locationData to the server
    def sendDataServer(self, idData, locationData):
        url = 'https://cpsdragonfly.herokuapp.com/api/item'
        auth = {'Authorization': '$2y$10$.wGNf1JYzHOjRF5xLFovSuEeJGk9XQ4hPaIGvY0D6jpaHR4Ib5/tO'}
        data = {'id': idData, 'location': locationData}
        response = requests.post(url, headers=auth, data=data) 
        print("SENT TO CPSDRAGONFLY-->  " + locationData + " " + idData)
