import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(26,GPIO.OUT) #Pin GPIO 3 kanan + 6 kanan 

buzzer=17 #Pin GPIO 6 kiri + 5 kiri
GPIO.setup(buzzer,GPIO.OUT)

from scipy.spatial import distance as dist

from imutils.video import FileVideoStream

from imutils.video import VideoStream
from imutils import face_utils
from azure.iot.device import IoTHubDeviceClient, Message

# import threading
import requests
import time
import os
import max30100
import serial
import pynmea2
# import datetime
import json
# import numpy as np
import argparse
import imutils
import dlib
import cv2

# connection_string = 'HostName=eye-detection.azure-devices.net;DeviceId=detection-eye;SharedAccessKey=37ghvA1i0FV+fnd/OKThBunqLlj0C7FsO8KkQAnB/3Y='

connection_string = "HostName=eye-detection.azure-devices.net;DeviceId=detection-eye;SharedAccessKey=37ghvA1i0FV+fnd/OKThBunqLlj0C7FsO8KkQAnB/3Y="

serial = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
serial.reset_input_buffer()
serial.flush()

device_client = IoTHubDeviceClient.create_from_connection_string(connection_string)

print('Connecting')
device_client.connect()
print('Connected')

TOTAL = 0
mx30 = max30100.MAX30100()
mx30.enable_spo2()

# #method EAR untuk deteksi mata
def eye_aspect_ratio(eye):

	A = dist.euclidean(eye[1], eye[5])
	B = dist.euclidean(eye[2], eye[4])

	C = dist.euclidean(eye[0], eye[3])
 
	ear = (A + B) / (2.0 * C)
 
	return ear

ap = argparse.ArgumentParser()
ap.add_argument("-p", "--shape-predictor", required=True,
	help="path to facial landmark predictor")
ap.add_argument("-v", "--video", type=str, default="",
	help="path to input video file")
args = vars(ap.parse_args())

EYE_AR_THRESH = 0.18
EYE_AR_CONSEC_FRAMES = 3

last_print_time = time.time()
bpm_interval = 1.0

COUNTER = 0
TOTAL = 0
bpm = 0
start_time = time.time()

print("[INFO] loading facial landmark predictor...")
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(args["shape_predictor"])

(lStart, lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
(rStart, rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]


print("[INFO] starting video stream thread...")
vs = FileVideoStream(args["video"]).start()
fileStream = True
vs = VideoStream(src=0).start()

fileStream = False
time.sleep(1.0)
        
def print_gps_data(line, bpm):
    msg = pynmea2.parse(line)
        
    if msg.sentence_type == 'GGA':
        lat = pynmea2.dm_to_sd(msg.lat)
        lon = pynmea2.dm_to_sd(msg.lon)

        if msg.lat_dir == 'S':
            lat = lat * -1

        if msg.lon_dir == 'W':
            lon = lon * -1
        
        message_json = { "gps" : { "lat":lat, "lon":lon, "bpm":bpm }}
        
        print("Sending telemetry", message_json)
        # message = Message(json.dumps(message_json))
        # device_client.send_message(message)
        
        # url = 'https://api.telegram.org/bot5513926646:AAEWbyK6_AxMWLyAElwWfoDpZ0lAnzPdl3E/sendMessage?chat_id=922321291&text="Sopir Mengantuk!"'
        url = "https://api.telegram.org/bot5513926646:AAEWbyK6_AxMWLyAElwWfoDpZ0lAnzPdl3E/sendMessage?chat_id=922321291&text=Sopir Mengantuk!. Lokasi Sopir http://www.google.com/maps/place/" + str(lat) + "," + str(lon)
        GPIO.output(26,GPIO.HIGH)
        
        GPIO.output(buzzer,GPIO.HIGH)
        time.sleep(1)
        GPIO.output(26, GPIO.LOW)
        GPIO.output(buzzer,GPIO.LOW)
        requests.get(url)      

#function menghitung moving average, untuk filter data yang tidak stabil/noise
def moving_average(numbers):
    window_size = 4
    i = 0
    # moving_averages = []
    while i < len(numbers) - window_size + 1:
        this_window = numbers[i : i + window_size]
        window_average = sum(this_window) / window_size
        i += 1
    try:
        return int((window_average/100))
    except:
        pass

#filter hasil yang didapat dari moving average
def display_filter(moving_average_bpm):
    try:
        if(moving_average_bpm<40):
            moving_average_bpm ='no beats found'
            moving_average_bpm = 0
        else:
            
            return moving_average_bpm
    except:
        return moving_average_bpm    

# def HR():
    
#         # elapsed_time = time.time() - start_time
#     mx30.read_sensor()
#     hb = int(mx30.ir / 100)
#     spo2 = int(mx30.red / 100)
#     if mx30.ir != mx30.buffer_ir :
#         moving_average_bpm = (moving_average(mx30.buffer_ir))
        
#     bpm = display_filter(moving_average_bpm)

#     # print("BPM : "+ str(bpm))
    
while True:
    elapsed_time = time.time() - start_time
    
    mx30.read_sensor()
    hb = int(mx30.ir / 100)
    spo2 = int(mx30.red / 100)
    if mx30.ir != mx30.buffer_ir :
        moving_average_bpm = (moving_average(mx30.buffer_ir))
        
        bpm = display_filter(moving_average_bpm)
        
        current_time = time.time()
        if current_time - last_print_time >= bpm_interval:
            print("BPM: " + str(bpm))
            last_print_time = current_time
    
    if fileStream and not vs.more():
        break
    
    frame = vs.read()
    frame = imutils.resize(frame, width=500)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    rects = detector(gray, 0)

    for rect in rects:
        shape = predictor(gray, rect)
        shape = face_utils.shape_to_np(shape)
        leftEye = shape[lStart:lEnd]
        rightEye = shape[rStart:rEnd]
        leftEAR = eye_aspect_ratio(leftEye)
        rightEAR = eye_aspect_ratio(rightEye)
        ear = (leftEAR + rightEAR) / 2.0

        leftEyeHull = cv2.convexHull(leftEye)
        rightEyeHull = cv2.convexHull(rightEye)
        cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
        cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)
        
        if ear < EYE_AR_THRESH:
            COUNTER += 1
            
        else:
            if COUNTER >= EYE_AR_CONSEC_FRAMES:
                TOTAL += 1
                   
                if elapsed_time < 60 and TOTAL >= 6 and bpm is not None and bpm > 40 and bpm <= 80:
                    
                    print("Mengantuk")
                    
                    TOTAL = 0
                    try:
                        line = serial.readline().decode('utf-8')

                        while len(line) > 0:
                            msg = pynmea2.parse(line)
                            if msg.sentence_type == 'GGA':
                                print_gps_data(line, bpm)
                                
                                break
                            
                            else:
                                line = serial.readline().decode('utf-8')
                            
                        time.sleep(5)
        
                    except UnicodeDecodeError:
                        line = serial.readline().decode('utf-8')
                        print("gagal")
                                    
                    TOTAL = 0
                    start_time = time.time()
                    
                elif elapsed_time > 60:
                    start_time = time.time()
                    TOTAL = 0
        
                COUNTER = 0	

        cv2.putText(frame, "Blinks: {}".format(TOTAL), (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(frame, "EAR: {:.2f}".format(ear), (300, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
    cv2.imshow("frame", frame)
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord("q"):
        break


cv2.destroyAllWindows()
vs.stop()
