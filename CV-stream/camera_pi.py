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
import operator
import queue as Queue
config = configparser.ConfigParser()
mmPerPixel = 0.245
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
# default if config file fails:
lower_blue = np.array([90,40,40])
upper_blue = np.array([160,255,255])
lower_red = np.array([90,40,40])
upper_red = np.array([160,255,255])
lower_white = np.array([90,40,40])
upper_white = np.array([160,255,255])

config.read('config.ini')
bL = config['blue']['HSVLOW']
bH = config['blue']['HSVHIGH']
rL = config['red']['HSVLOW']
rH = config['red']['HSVHIGH']
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

	maskB = cv2.inRange(hsv, lower_blue, upper_blue)
	maskR = cv2.inRange(hsv, lower_red, upper_red)
	maskW = cv2.inRange(hsv, lower_white, upper_white)
	blueArea = cv2.countNonZero(maskB)
	redArea = cv2.countNonZero(maskR)
	whiteArea = cv2.countNonZero(maskW)
	#print(blueArea, redArea, whiteArea)
	colors = {'blue':blueArea, 'red':redArea, 'white': whiteArea}
	maxColor = 'no color'
	maxColor = max(colors.items(), key=operator.itemgetter(1))[0]
	#print("maxColor: ", maxColor)
	#print("Dominant color: ", maxColor)
	color = (10,10,10)
	if maxColor == 'blue':
		mask = maskB
		color = (255,0,0)
	elif maxColor == 'red':
		mask = maskR
		color = (0,0,255)
	elif maxColor == 'white':
		mask = maskW
		color = (255,255,255)

	res = cv2.bitwise_and(mat,mat, mask= mask)
	# Threshold the HSV image to get only blue colors
	#mask = cv2.inRange(hsv, lower_blue, upper_blue)
	#_, contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	#contours = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	contours = contours[1]
	#contours = contours[1]
	#hierarchy = hierarchy[0]
	#for component in zip(contours, hierarchy):
	areaMax = 0
	maxContour = None
	foundObject = False
	imData = [foundObject,0, 0, maxColor]
	for c in contours:
		# compute the center of the contour
		#c = component[0]
		#currentHierarchy = component[1]
		#if currentHierarchy[2] == -1:
		#	pass
		#else:
		area = cv2.contourArea(c)
		if area > 2000:
			foundObject = True
			#print("Contour area: ", area )
			M = cv2.moments(c)
			if M["m00"] != 0:
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
			else:
				cX = 0
				cY = 0

			# draw the contour and center of the shape on the image
			cv2.drawContours(mat, [c], -1, color, 2)
			cv2.circle(mat, (cX, cY), 7, (255, 255, 255), -1)
			cv2.putText(mat, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
			# find largest contour:
			if area>areaMax:
				areaMax = area
				maxContour = c
				imData = [foundObject, cX, cY, maxColor]
				#print(cX, cY)

	return mat, imData

def pix2mm(pixel):
	# camera coord -> robot coord
	return mmPerPixel*(pixel-165)
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
			camera.exposure_compensation = -10
			q = Camera.queue
			time.sleep(2)
			camera.resolution = (640,480)
			#stream = io.BytesIO()
			stream = PiRGBArray(camera, size=(640, 480))
			print("frames function called, capturing frames:")
			counter = 0
			counterMax = 20 # How many coordinates to put in the queue
			y_start = 100
			y_end = 200 # reset counter
			prevX = 0 # check consistency of measurements
			prevY = 0
			xErrorMargin = 10
			yDelta = 30
			positionsList = [ ]
			for frame in camera.capture_continuous(stream, 'bgr',  use_video_port=True):
				# return current frame
				#stream.seek(0)
				#mat = stream.read()
				mat = frame.array
				# Crop image
				mat = mat[0:280, 140:470]
				mat, imDat = imgproc(mat)
				found = imDat[0]
				xPos = imDat[1]
				yPos = imDat[2]
				color = imDat[3]
				if found == True:
						# Check ( consistency )
					if abs(xPos-prevX) < xErrorMargin and (yPos >= prevY-yDelta):
						if counter < counterMax:
							#q.put([xPos,yPos,color])
							positionsList.append(xPos)
							counter += 1
							if counter == counterMax:
								# Success: we had the desired amount of consequent measurements
								position = np.mean(positionsList)
								position = pix2mm(position)
								q.put([position, color])
					# Discard and start again
					else:
						counter = 0
						positionsList = []
					# only update positions if found
					prevX = xPos
					prevY = yPos

				streamImage = cv2.imencode('.jpg', mat)[1].tobytes()


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
				yield streamImage
			cv2.destroyAllWindows()
