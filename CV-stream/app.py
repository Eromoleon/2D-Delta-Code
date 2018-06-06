#!/usr/bin/env python

from importlib import import_module
import os
from flask import Flask, render_template, Response, flash, request
from wtforms import Form, TextField, TextAreaField, validators, StringField, SubmitField
import time
import threading
import queue as Queue
import serial
import subprocess
from io import StringIO
import csv

time.sleep(2)

# Set up Serial communication with arduino:
BAUDRATE = 115200
ser = serial.Serial('/dev/ttyACM0', BAUDRATE) # Establish the connection on a specific port
ser.timeout = 10

# import the camera:
Camera = import_module('camera_' + 'pi').Camera

# Initialize the queues used to communicate between the threads:
# q = Queue.Queue()
commandQ = Queue.Queue()
PositionsQ = Queue.Queue()



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
	camera = Camera(PositionsQ)
else: camera = None

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
				pass
				#print("Queue empty")
				#time.sleep(5)
				#self._stopevent.wait(self._sleepperiod)
			else:
				command = commandQ.get()
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


def commandManager(command):

	commandList = parseCommand(command)
	determinant = commandList[0]
	if determinant == 0:
		normal()
	elif determinant == 1:
		directKin(command)
	elif determinant == 2:
		inverseKin(command)
	else:
		print("Command not recognised")
	return True


def normal():
	print('Normal operation')
	return 0
def directKin(command):
	print('Direct kinematics')
	print( command)
	print ('thetaL: ' , thetaL, '\nthetaR:',thetaR, '\n' )
	return 0
def inverseKin(command):
	print('Inverse kinematics')

	print( command)
	#ser.write(command.encode())
	command = command+'\n'
	com = '2;0;140\n'
	encoded = command.encode('ascii')
	ser.write(encoded)
	return True


arduThread = arduinoThread(1, "Thread-1", 1, commandQ)

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
	arduThread.join()
	camera.stopNow()
	return ("Stopping")


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
    # debug:
    #for number in commandNumbers:
    #    print(number)
    return commandNumbers

if __name__ == '__main__':


	arduThread.start()
	print(threading.activeCount())

	if eyeless == False:
		gen(camera)

	print(threading.activeCount())
	#time.sleep(2)
	if disp == False:
		#if no display is connected use the remote UI:
		app.run(host='0.0.0.0', threaded=True, debug=False)
	else:
		pass


	arduThread.join()
	print("Arduino thread stopped")

