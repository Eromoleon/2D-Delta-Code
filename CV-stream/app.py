#!/usr/bin/env python

from importlib import import_module
import os
from flask import Flask, render_template, Response, flash, request
from wtforms import Form, TextField, TextAreaField, validators, StringField, SubmitField
import time
import threading
import queue as Queue
import serial
import serial.tools.list_ports
import subprocess
from io import StringIO
import csv
import serial.tools.list_ports
import numpy as np
time.sleep(2)
cameraOnly = True # Debugging the camera only and turn everything else off
# Automatically find arduino based on hardware indentification if connected
ports = list(serial.tools.list_ports.comports())
arduPort = ''
for p in ports:
    if (' VID:PID=2341:0043'in p.hwid):
        print("Arduino detected with hardware id: ", p.hwid)
        arduPort = '/dev/'+p.description
arduConnect = False
if(arduPort == ''):
	#raise ValueError('No arduino connected. Please check your device connections')
    print("WARNING: No arduino connected. Please check your device connections")
else:
    print("Arduino connected at: ", arduPort)
    arduConnect = True


# Set up Serial communication with arduino:
BAUDRATE = 115200
#ser = serial.Serial('/dev/ttyACM0', BAUDRATE) # Establish the connection on a specific port

if arduConnect == True:
    ser = serial.Serial(arduPort, BAUDRATE) # Establish the connection on a specific port
    ser.timeout = 10
else: ser = None
# import the camera:
Camera = import_module('camera_' + 'pi').Camera

# Initialize the queues used to communicate between the threads:
# This queue buffers the command sent from the web. It is the interface
# between the webserver and the command manager (arduino manager) thread.
commandQ = Queue.Queue()

# This queue buffers the image positions found by the image processing thread.
# This is the interface between the image processing thread and
# the command Manager (Arduino manager) thread.
positionsQ = Queue.Queue()
# Check display connection:
# True if a display (real or virtual like VNC) is connected.
disp = False
try:
	d = os.environ['DISPLAY']
except:
	print("No display connected...")
else:
	disp = True

# Check camera connection:
eyeless = True
c = subprocess.check_output(["vcgencmd","get_camera"])
if c==b'supported=1 detected=0\n':
	print('No camera detected')
	eyeless = True
else:
	print('Camera detected')
	eyeless = False

if eyeless == False:
    try:
	       camera = Camera(positionsQ)
    except:
        raise IOError("Another application is already using the camera.")
else: camera = None

# Initialize command manager thread:
class arduinoThread (threading.Thread):
    def __init__(self, threadID, name, counter, q):
    	threading.Thread.__init__(self)
    	self.threadID = threadID
    	self.name = name
    	self.counter = counter
    	self._stopevent = threading.Event( )
    def run(self):
    	print ("Starting " + self.name)
    	#print_time(self.name, self.counter, 5)
    	## arduino management activity:
    	print( "Command Manager Started")
    	while not self._stopevent.isSet( ):
            if commandQ.empty():
                time.sleep(0.01)
                pass
            else:
                time.sleep(0.01)
                command = commandQ.get()
                if arduConnect == True:
                    ret = commandManager(command)
                    if ret == True:
                    	print('Command executed successfully')
                    else:
                    	print('Command failed')
                    	#time.sleep(5)

    def join(self):
    	""" Stop the thread and wait for it to end. """
    	timeout = 2
    	self._stopevent.set( )
    	threading.Thread.join(self, timeout)
    	print ("Exiting " + self.name)

# Define command manager function: TODO: Rename task manager?
def commandManager(command):

    commandList = parseCommand(command)
    determinant = commandList[0]
    if determinant == 0:
    	normal()
    elif determinant == 1:
    	directKin(command)
    elif determinant == 2:
        inverseKin(command)
    elif determinant == 4 or determinant == 3:
        command = command+'\n'
        #encoded = command.encode('ascii')
        encoded = command.encode()
        ret = ser.write(encoded)
        response = ser.readline()
        if response == 1:
            print("Movement executed successfully.")
        if response == -1:
            print("Position is out of range.")
        print("Command sent to serial")
    else:
    	print("Command not recognised")
    return True

def normal():
    print('Normal operation')
    with positionsQ.mutex:
        positionsQ.queue.clear()
    while(commandQ.empty()):
        if(positionsQ.empty()):
            time.sleep(2)
        else:
            # start the conveyor as soon as an object is detected
            command = '1;90;90;0;1\n' # return to start position and start the conveyor
            encoded = command.encode()
            ret = ser.write(encoded)
            response = ser.readline()
            # get the image data and move robot acccordingly:
            imData = positionsQ.get()
            xPos = int(round( imData[0] ))
            colorInt = 0
            color = imData[1]
            if color == 'blue':
                colorInt = 0
            elif color == 'red':
                colorInt = 1
            elif color == 'white':
                colorInt = 2

            print("Received position", xPos, "Color: ", color)

            commandList = [0, xPos, colorInt ]
            command = ';'.join(map(str, commandList))
            command = command + '\n'
            print(command)
            encoded = command.encode()
            ret = ser.write(encoded)
            response = ser.readline()
            if response == 1:
                print("Movement executed successfully.")
            if response == -1:
                print("Error at normal operation.")

    return 0

def directKin(command):
	print('Direct kinematics')
	print( command)
	command = command + '\n'
	encoded = command.encode()
	ret = ser.write(encoded)
	print('Command sent to serial')
	print(ret)
	return True

def inverseKin(command):
    print('Inverse kinematics')

    print( command)
    #command = 'b'+command+'\n'
    command = command+'\n'
    #encoded = command.encode('ascii')
    encoded = command.encode()
    ret = ser.write(encoded)
    response = ser.readline()
    if response == 1:
        print("Movement executed successfully.")
    if response == -1:
        print("Position is out of range.")
    print("Command sent to serial")
    return True

arduThread = arduinoThread(1, "Thread-1", 1, commandQ)

 # Define web server:
app = Flask(__name__)
@app.route('/')
def index():
	"""Video streaming home page."""
	if eyeless == False:
		return render_template('index.html')
	else:
		return render_template('index_noCam.html')

def gen(camera):
    """Video streaming generator function."""
    if eyeless == False:
        camera.startNow()
        while True:
            frame = camera.get_frame()
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
	"""Video streaming route. Put this in the src attribute of an img tag."""
	#camera = Camera(q) # initialize the camera with the queue

	camera.startNow() # remove the stop flag if it was set
	return Response(gen(camera),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/stop')
def stoprequest():
	print("Stop called")
	#arduThread.join()
	camera.stopNow()
	return ("Stopping")

@app.route('/start')
def startrequest(): # doesn't work :()
    print("start called")
    #arduThread.join()
    camera.startNow()
    gen(camera)
    return ("Starting")

@app.route('/gui')
def my_form():
    return render_template('WebGUI.html')

@app.route('/gui', methods=['POST'])
def my_form_post():
	command = request.form['text']
	#print(command)


	#for number in commandList:
	#	print(number)
	commandQ.put(command)
	return "Command received: %s" % (command)



@app.route('/api',methods = ['POST', 'GET'])
def base():
	if request.method == 'POST':
		command = request.form['command'] # csv string

		print('Received command (CSV):')
		print(command)

		#print('Numbers parsed from command CSV:')
		#for number in commandList:
		#	print (number)
		commandQ.put(command)

		return "Command received: %s" % (command)
	else:
	#get request
		return "Status: "

# END of flask functions^^

# Non flask functions:

def parseCommand(command):
    'Takes a csv string and converts it into a list of floating point variables. Requires StringIO and csv modules'
    #command = "1;32;150;0"
    f = StringIO(command)
    reader = csv.reader(f, delimiter=';')
    comList = list(reader)
    commandNumbers = list(map(float, comList[0]))
    return commandNumbers

if __name__ == '__main__':

    camera.startProcessing()
    #if cameraOnly == False:
    arduThread.start()
    print(threading.activeCount())
    if eyeless == False:
    	gen(camera)
    print(threading.activeCount())
    #time.sleep(2)
      #if disp == False:
    	#if no display is connected use the remote UI:
    #if cameraOnly == False:
    app.run(host='0.0.0.0', threaded=True, debug=False)
    #else:
    #	pass
    #if cameraOnly == False:
    arduThread.join()
    print("Arduino thread stopped")
