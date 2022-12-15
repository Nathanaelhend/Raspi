import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18,GPIO.OUT) #Pin GPIO 3 kanan + 6 kanan 

buzzer=17 #Pin GPIO 6 kiri + 5 kiri
GPIO.setup(buzzer,GPIO.OUT)

from scipy.spatial import distance as dist


from imutils.video import FileVideoStream
from telepot.loop import MessageLoop
from imutils.video import VideoStream
from imutils import face_utils
from azure.iot.device import IoTHubDeviceClient, Message
from adafruit_ads1x15.analog_in import AnalogIn

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
import requests
import telepot
import time
import serial
import pynmea2
import datetime
import json
import numpy as np
import argparse
import imutils
import dlib
import cv2

i2c = busio.I2C(board.SCL, board.SDA)

adc = ADS.ADS1115(i2c)
# chan = AnalogIn(ads, ADS.P0)
# initialization 
GAIN = 2/3
curState = 0
thresh = 525  # mid point in the waveform
P = 512
T = 512
stateChanged = 0
sampleCounter = 0
lastBeatTime = 0
firstBeat = True
secondBeat = False
Pulse = False
IBI = 600
rate = [0]*10
amp = 100

lastTime = int(time.time()*1000)
    
connection_string = 'HostName=eye-detection.azure-devices.net;DeviceId=detection-eye;SharedAccessKey=37ghvA1i0FV+fnd/OKThBunqLlj0C7FsO8KkQAnB/3Y='

serial = serial.Serial('/dev/ttyAMA0')
serial.reset_input_buffer()
serial.flush()

device_client = IoTHubDeviceClient.create_from_connection_string(connection_string)

print('Connecting')
device_client.connect()
print('Connected')


TOTAL = 0

adc = ADS.ADS1115(i2c)
    # chan = AnalogIn(ads, ADS.P0)
    # initialization 
GAIN = 2/3
curState = 0
thresh = 525  # mid point in the waveform
P = 512
T = 512
stateChanged = 0
sampleCounter = 0
runningTotal = 0
lastBeatTime = 0
firstBeat = True
secondBeat = False
Pulse = False
IBI = 600
rate = [0]*10
amp = 100

lastTime = int(time.time()*1000)

#method untuk gps 
def print_gps_data(line):
    msg = pynmea2.parse(line)
    if msg.sentence_type == 'GGA':
        lat = pynmea2.dm_to_sd(msg.lat)
        lon = pynmea2.dm_to_sd(msg.lon)

        if msg.lat_dir == 'S':
            lat = lat * -1

        if msg.lon_dir == 'W':
            lon = lon * -1


        message_json = { "gps" : { "lat":lat, "lon":lon } }
        print("Sending telemetry", message_json)
        message = Message(json.dumps(message_json))
        device_client.send_message(message)
        time.sleep(5)
		
#method EAR untuk deteksi mata
def eye_aspect_ratio(eye):
    	# compute the euclidean distances between the two sets of
	# vertical eye landmarks (x, y)-coordinates
	A = dist.euclidean(eye[1], eye[5])
	B = dist.euclidean(eye[2], eye[4])
	# compute the euclidean distance between the horizontal
	# eye landmark (x, y)-coordinates
	C = dist.euclidean(eye[0], eye[3])
	# compute the eye aspect ratio
	ear = (A + B) / (2.0 * C)
	# return the eye aspect ratio
	return ear

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--shape-predictor", required=True,
	help="path to facial landmark predictor")
ap.add_argument("-v", "--video", type=str, default="",
	help="path to input video file")
args = vars(ap.parse_args())

# define two constants, one for the eye aspect ratio to indicate
# blink and then a second constant for the number of consecutive
# frames the eye must be below the threshold
EYE_AR_THRESH = 0.3
EYE_AR_CONSEC_FRAMES = 3
# initialize the frame counters and the total number of blinks
COUNTER = 0



# initialize dlib's face detector (HOG-based) and then create
# the facial landmark predictor
print("[INFO] loading facial landmark predictor...")
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(args["shape_predictor"])

# grab the indexes of the facial landmarks for the left and
# right eye, respectively
(lStart, lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
(rStart, rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]

# start the video stream thread
print("[INFO] starting video stream thread...")
vs = FileVideoStream(args["video"]).start()
fileStream = True
vs = VideoStream(src=0).start()
# vs = VideoStream(usePiCamera=True).start()
fileStream = False
time.sleep(1.0)
    
# loop over frames from the video stream
while True:
	# if this is a file video stream, then we need to check if
	# there any more frames left in the buffer to process
	if fileStream and not vs.more():
		break
	# grab the frame from the threaded video file stream, resize
	# it, and convert it to grayscale
	# channels)
	frame = vs.read()
	frame = imutils.resize(frame, width=450)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	# detect faces in the grayscale frame
	rects = detector(gray, 0)

 	# loop over the face detections
	for rect in rects:
		# determine the facial landmarks for the face region, then
		# convert the facial landmark (x, y)-coordinates to a NumPy
		# array
		shape = predictor(gray, rect)
		shape = face_utils.shape_to_np(shape)
		# extract the left and right eye coordinates, then use the
		# coordinates to compute the eye aspect ratio for both eyes
		leftEye = shape[lStart:lEnd]
		rightEye = shape[rStart:rEnd]
		leftEAR = eye_aspect_ratio(leftEye)
		rightEAR = eye_aspect_ratio(rightEye)
		# average the eye aspect ratio together for both eyes
		ear = (leftEAR + rightEAR) / 2.0
  
  # compute the convex hull for the left and right eye, then
		# visualize each of the eyes
		leftEyeHull = cv2.convexHull(leftEye)
		rightEyeHull = cv2.convexHull(rightEye)
		cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
		cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)
	
  
  # check to see if the eye aspect ratio is below the blink
		# threshold, and if so, increment the blink frame counter
		if ear < EYE_AR_THRESH:
			COUNTER += 1
		# otherwise, the eye aspect ratio is not below the blink
		# threshold
		else:
			# if the eyes were closed for a sufficient number of
			# then increment the total number of blinks
			if COUNTER >= EYE_AR_CONSEC_FRAMES:
				TOTAL += 1

				if TOTAL >= 5:
					
					def print_gps_data(line):
						msg = pynmea2.parse(line)
						if msg.sentence_type == 'GGA':
							lat = pynmea2.dm_to_sd(msg.lat)
							lon = pynmea2.dm_to_sd(msg.lon)

							if msg.lat_dir == 'S':
								lat = lat * -1

							if msg.lon_dir == 'W':
								lon = lon * -1


							message_json = { "gps" : { "lat":lat, "lon":lon } }
							print("Sending telemetry", message_json)
							message = Message(json.dumps(message_json))
							device_client.send_message(message)

							time.sleep(5)
							# url = 'https://api.telegram.org/bot5513926646:AAEWbyK6_AxMWLyAElwWfoDpZ0lAnzPdl3E/sendMessage?chat_id=922321291&text="Sopir Mengantuk!"'
      					
							# GPIO.output(18,GPIO.HIGH)
							# GPIO.output(buzzer,GPIO.HIGH)
							# time.sleep(1)
							# GPIO.output(18, GPIO.LOW)
							# GPIO.output(buzzer,GPIO.LOW)
							# requests.get(url)
							# time.sleep(5)
					
					try:
						line = serial.readline().decode('utf-8')

						while len(line) > 0:
							print_gps_data(line)
							line = serial.readline().decode('utf-8')
							time.sleep(5)
							
            
					except UnicodeDecodeError:
						line = serial.readline().decode('utf-8')
					
     
					# url = 'https://api.telegram.org/bot5513926646:AAEWbyK6_AxMWLyAElwWfoDpZ0lAnzPdl3E/sendMessage?chat_id=922321291&text="Sopir Mengantuk!"'
      					
					# GPIO.output(18,GPIO.HIGH)
					# GPIO.output(buzzer,GPIO.HIGH)
					# time.sleep(1)
					# GPIO.output(18, GPIO.LOW)
					# GPIO.output(buzzer,GPIO.LOW)
					# requests.get(url)
					# time.sleep(5)
					# break
					# print_gps_data(line)
					# line = serial.readline().decode('utf-8')
					# print_gps_data(line)
					
					if len(line)> 0:
						print_gps_data(line)
						line = serial.readline().decode('utf-8')
					url = 'https://api.telegram.org/bot5513926646:AAEWbyK6_AxMWLyAElwWfoDpZ0lAnzPdl3E/sendMessage?chat_id=922321291&text="Sopir Mengantuk!"'
					GPIO.output(18,GPIO.HIGH)
					GPIO.output(buzzer,GPIO.HIGH)
					time.sleep(1)
					GPIO.output(18, GPIO.LOW)
					GPIO.output(buzzer,GPIO.LOW)
					requests.get(url)
     
					
					# time.sleep(5)

					TOTAL = 0
     				# device_client.send_message(TOTAL)
        			# print_gps_data(line)
					
					
			# reset the eye frame counter
			COUNTER = 0	
   
   		# draw the total number of blinks on the frame along with
		# the computed eye aspect ratio for the frame
		cv2.putText(frame, "Blinks: {}".format(TOTAL), (10, 30),
			cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
		cv2.putText(frame, "EAR: {:.2f}".format(ear), (300, 30),
			cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
					
	# show the frame
	cv2.imshow("frame", frame)
	key = cv2.waitKey(1) & 0xFF
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

cv2.destroyAllWindows()
vs.stop()

# while True:
#         # read from the ADC
#         Signal = AnalogIn(adc, ADS.P0)   #TODO: Select the correct ADC channel. I have selected A0 here
#         curTime = int(time.time()*1000)

#         sampleCounter += curTime - lastTime;      #                   # keep track of the time in mS with this variable
#         lastTime = curTime
#         N = sampleCounter - lastBeatTime;     #  # monitor the time since the last beat to avoid noise
#         #print N, Signal, curTime, sampleCounter, lastBeatTime

#         ##  find the peak and trough of the pulse wave
#         if Signal.value < thresh and N > (IBI/5.0)*3.0 :  #       # avoid dichrotic noise by waiting 3/5 of last IBI
#             if Signal.value < T :                        # T is the trough
#               T = Signal.value;                         # keep track of lowest point in pulse wave 

#         if Signal.value > thresh and  Signal.value > P:           # thresh condition helps avoid noise
#             P = Signal.value;                             # P is the peak
#                                                 # keep track of highest point in pulse wave

#           #  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
#           # signal surges up in value every time there is a pulse
#         if N > 250 :                                   # avoid high frequency noise
#             if  (Signal.value > thresh) and  (Pulse == False) and  (N > (IBI/5.0)*3.0)  :       
#               Pulse = True;                               # set the Pulse flag when we think there is a pulse
#               IBI = sampleCounter - lastBeatTime;         # measure time between beats in mS
#               lastBeatTime = sampleCounter;               # keep track of time for next pulse

#               if secondBeat :                        # if this is the second beat, if secondBeat == TRUE
#                 secondBeat = False;                  # clear secondBeat flag
#                 for i in range(0,10):             # seed the running total to get a realisitic BPM at startup
#                   rate[i] = IBI;                      

#               if firstBeat :                        # if it's the first time we found a beat, if firstBeat == TRUE
#                 firstBeat = False;                   # clear firstBeat flag
#                 secondBeat = True;                   # set the second beat flag
#                 continue                              # IBI value is unreliable so discard it


#               # keep a running total of the last 10 IBI values
#               runningTotal = 0;                  # clear the runningTotal variable    

#               for i in range(0,9):                # shift data in the rate array
#                 rate[i] = rate[i+1];                  # and drop the oldest IBI value 
#                 runningTotal += rate[i];              # add up the 9 oldest IBI values

#               rate[9] = IBI;                          # add the latest IBI to the rate array
#               runningTotal += rate[9];                # add the latest IBI to runningTotal
#               runningTotal /= 10;                     # average the last 10 IBI values 
#               BPM = 60000/runningTotal;               # how many beats can fit into a minute? that's BPM!
#               print ('BPM: {}'.format(int(BPM)))
#               time.sleep(1)

#         if Signal.value < thresh and Pulse == True :   # when the values are going down, the beat is over
#             Pulse = False;                         # reset the Pulse flag so we can do it again
#             amp = P - T;                           # get amplitude of the pulse wave
#             thresh = amp/2 + T;                    # set thresh at 50% of the amplitude
#             P = thresh;                            # reset these for next time
#             T = thresh;

#         if N > 2500 :                          # if 2.5 seconds go by without a beat
#             thresh = 512;                          # set thresh default
#             P = 512;                               # set P default
#             T = 512;                               # set T default
#             lastBeatTime = sampleCounter;          # bring the lastBeatTime up to date        
#             firstBeat = True;                      # set these to avoid noise
#             secondBeat = False;                    # when we get the heartbeat back
#             print ("no beats found")

#         time.sleep(0.005)

    
# try:
#     line = serial.readline().decode('utf-8')

#     while len(line) > 0:
#     	# print_gps_data(line)
#         line = serial.readline().decode('utf-8')
        
# except UnicodeDecodeError:
#     line = serial.readline().decode('utf-8')
# do a bit of cleanup


