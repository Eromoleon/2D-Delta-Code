import io
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
from base_camera import BaseCamera
import cv2
import numpy as np
import os
import configparser
import json
config = configparser.ConfigParser()
# For creating trackbars:
def nothing(x):
	pass
disp = False # True if a display (real or virtual like VNC) is connected.
try:
	d = os.environ['DISPLAY']
except:
	print("No display connected...")
else:
	disp = True

lower_blue = np.array([90,40,40])
upper_blue = np.array([160,255,255])
lower_red = np.array([90,40,40])
upper_red = np.array([160,255,255])
lower_white = np.array([90,40,40])
upper_white = np.array([160,255,255])

config.read('config.ini')
bL = config['blue']['HSVLOW']
bH = config['blue']['HSVHIGH']
rL = config['red']['HSVHIGH']
rH = config['red']['HSVLOW']
wL = config['white']['HSVLOW']
wH = config['white']['HSVHIGH']

lower_blue = np.array(json.loads(bL))
upper_blue = np.array(json.loads(bH))
lower_red = np.array(json.loads(rL))
upper_red = np.array(json.loads(rH))
lower_white = np.array(json.loads(wL))
upper_white = np.array(json.loads(wH))

def imgproc(mat):
	hsv = cv2.cvtColor(mat, cv2.COLOR_BGR2HSV)
	# define range of blue color in HSV
	mask = np.zeros_like(hsv)
	mask = cv2.inRange(hsv, lower_blue, upper_blue)

	# Threshold the HSV image to get only blue colors
	#mask = cv2.inRange(hsv, lower_blue, upper_blue)
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[1]
	# Bitwise-AND mask and original image
	for c in cnts:
		# compute the center of the contour
		M = cv2.moments(c)
		if M["m00"] != 0:
			cX = int(M["m10"] / M["m00"])
			cY = int(M["m01"] / M["m00"])
		else:
			cX = 0
			cY = 0

		# draw the contour and center of the shape on the image
		cv2.drawContours(mat, [c], -1, (0, 255, 0), 2)
		cv2.circle(mat, (cX, cY), 7, (255, 255, 255), -1)
		cv2.putText(mat, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

	res = cv2.bitwise_and(mat,mat, mask= mask)
	return mat

class Camera(BaseCamera):
	@staticmethod
	def stop():
		Camera.stop = True

	@staticmethod
	def frames():
		#Camera._stopProcessing.clear()
		# Determine if there is a display connected:
		disp = False # True if a display (real or virtual like VNC) is connected.
		try:
			d = os.environ['DISPLAY']
		except:
			pass
		else:
			disp = True

		if Camera._stopProcessing.isSet():
			print("processing paused")
		else:
			print("Processing started now.")

		with PiCamera() as camera:
			# let camera warm up
			Camera._stopevent.clear()
			q = Camera.queue
			time.sleep(2)
			camera.resolution = (640,480)
			#stream = io.BytesIO()
			stream = PiRGBArray(camera, size=(640, 480))
			print("frames function called, capturing frames:")
			for frame in camera.capture_continuous(stream, 'bgr',  use_video_port=True):
				# return current frame
				#stream.seek(0)
				#mat = stream.read()
				mat = frame.array
				mat = mat[0:480, 140:470]

				if Camera._stopProcessing.isSet():
					pass
				else:
					pass
					mat = imgproc(mat)
				streamImage = cv2.imencode('.jpg', mat)[1].tobytes()
				q.put((0,0))
				yield streamImage
				# reset stream for next frame
				stream.seek(0)
				stream.truncate()
				key = None
				if disp == True:
					cv2.imshow("Processed image", mat)

					key = cv2.waitKey(1) & 0xFF
				if disp == True and key == ord("q"):
					Camera.stopNow()
					break
			cv2.destroyAllWindows()
