#!/usr/bin/env python
from importlib import import_module
import os
from flask import Flask, render_template, Response, flash, request
from wtforms import Form, TextField, TextAreaField, validators, StringField, SubmitField
import time
import threading
import queue as Queue
time.sleep(2)
Camera = import_module('camera_' + 'pi').Camera
q = Queue.Queue()
camera = Camera(q)

# True if a display (real or virtual like VNC) is connected.
disp = False 
try:
	d = os.environ['DISPLAY']
except:
	print("No display connected...")
else:
	disp = True

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
		#arduinoManager(self.name,self.counter, q)
		
		## arduino management activity:
		while not self._stopevent.isSet( ):
			print( "Arduino Manager Running")
			if q.empty():
				print("Queue empty")
				time.sleep(5)
				self._stopevent.wait(self._sleepperiod)
			else:
				print (q.get())
				time.sleep(5)
				
	def join(self):
		""" Stop the thread and wait for it to end. """
		timeout = 2
		self._stopevent.set( )
		threading.Thread.join(self, timeout)
		print ("Exiting " + self.name)
		
arduThread = arduinoThread(1, "Thread-1", 1, q)		

def arduinoManager(threadName, counter, q):
	while True:
		print( "Arduino Manager Running")
		if q.empty():
			print("Queue empty")
			time.sleep(5)
		else:
			print (q.get())
			time.sleep(5)

	

app = Flask(__name__)
@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')


def gen(camera):
    """Video streaming generator function."""
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
	"""Video streaming route. Put this in the src attribute of an img tag."""
	camera = Camera(q) # initialize the camera with the queue
	camera.startNow() # remove the stop flag if it was set
	return Response(gen(camera),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

					
@app.route('/stop')
def stoprequest():
	print("Stop called")
	arduThread.join()
	camera.stopNow()
	
	return ("Stopping")
	
	
@app.route('/api',methods = ['POST', 'GET'])
def base():
	if request.method == 'POST':
		command = request.form['command'] # csv string
		return "Command received: %s" % (command)
	else:
	#get request
		return "Status: "
	
if __name__ == '__main__':

	
	arduThread.start()
	print(threading.activeCount())
	#camera = Camera(q)
	#gen(Camera(q))
	gen(camera)
	print(threading.activeCount())
	#time.sleep(2)
	app.run(host='0.0.0.0', threaded=True, debug=False)
	arduThread.join()
	print("Arduino thread stopped")
	
	
