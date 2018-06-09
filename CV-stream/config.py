import io
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
from base_camera import BaseCamera
import cv2
import numpy as np
import os
import configparser
config = configparser.ConfigParser()

HSVLOW = None
HSVHIGH = None

# Default values, todo: optimize
lower_blue = np.array([90,40,40])
upper_blue = np.array([160,255,255])
lower_red = np.array([90,40,40])
upper_red = np.array([160,255,255])
lower_white = np.array([90,40,40])
upper_white = np.array([160,255,255])
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


windowName = 'Thresholds'
cv2.namedWindow(windowName)
hh='Hue High'
hl='Hue Low'
sh='Saturation High'
sl='Saturation Low'
vh='Value High'
vl='Value Low'
switch = '0 : Blue \n1 : Red\n2: White'
#Begin Creating trackbars for each
cv2.createTrackbar(hl, windowName,0,179,nothing)
cv2.createTrackbar(hh, windowName,0,179,nothing)
cv2.createTrackbar(sl, windowName,0,255,nothing)
cv2.createTrackbar(sh, windowName,0,255,nothing)
cv2.createTrackbar(vl, windowName,0,255,nothing)
cv2.createTrackbar(vh, windowName,0,255,nothing)
cv2.createTrackbar(switch, windowName,0,2,nothing)

def imgproc(mat):
	hsv = cv2.cvtColor(mat, cv2.COLOR_BGR2HSV)
	# define range of blue color in HSV
	mask = np.zeros_like(hsv)
	mask = cv2.inRange(hsv, HSVLOW, HSVHIGH)

	res = cv2.bitwise_and(mat,mat, mask= mask)
	return mat


if __name__ == '__main__':
    if disp == True:
        with PiCamera() as camera:
            # let camera warm up
            camera.resolution = (640,480)
            #stream = io.BytesIO()
            stream = PiRGBArray(camera, size=(640, 480))
            print("Calibration script running")
            for frame in camera.capture_continuous(stream, 'bgr',  use_video_port=True):
                mat = frame.array
                mat = mat[0:480, 140:470] # Crop image to see only the conveyor belt
                hsv = cv2.cvtColor(mat, cv2.COLOR_BGR2HSV)
                mask = np.zeros_like(hsv)
                # reset stream for next frame
                stream.seek(0)
                stream.truncate()
                key = None
                key = cv2.waitKey(1) & 0xFF
                # Save thresholds:

                hul=cv2.getTrackbarPos(hl, windowName)
                huh=cv2.getTrackbarPos(hh, windowName)
                sal=cv2.getTrackbarPos(sl, windowName)
                sah=cv2.getTrackbarPos(sh, windowName)
                val=cv2.getTrackbarPos(vl, windowName)
                vah=cv2.getTrackbarPos(vh, windowName)
                s = cv2.getTrackbarPos(switch,windowName)
                #make array for final values
                HSVLOW=np.array([hul,sal,val])
                HSVHIGH=np.array([huh,sah,vah])
                # define range of blue color in HSV

                mask = cv2.inRange(hsv, HSVLOW, HSVHIGH)
                if s ==0:
                    upper_blue = HSVHIGH
                    lower_blue = HSVLOW
                elif s == 1:
                    upper_red = HSVHIGH
                    lower_red = HSVLOW
                elif s == 2:
                    upper_white = HSVHIGH
                    lower_white = HSVLOW

                res = cv2.bitwise_and(mat,mat, mask= mask)
                cv2.imshow(windowName,mask)
                cv2.imshow("Processed image", res)

                if key == ord("s"):

                    print("Saving thesholds to calibration file")

                    config['blue'] = {}
                    config['blue']['HSVLOW'] = np.array2string(lower_blue,  separator=',')
                    config['blue']['HSVHIGH'] = np.array2string(upper_blue,  separator=',')
                    config['red'] = {}
                    config['red']['HSVLOW'] = np.array2string(lower_red,  separator=',')
                    config['red']['HSVHIGH'] = np.array2string(upper_red,  separator=',')
                    config['white'] = {}
                    config['white']['HSVLOW'] = np.array2string(lower_white,  separator=',')
                    config['white']['HSVHIGH'] = np.array2string(upper_white,  separator=',')

                    with open('config.ini', 'w') as configfile:
                        config.write(configfile)



                if key == ord("q"):
                    break
            cv2.destroyAllWindows()
    else:
        raise ValueError("No display connected. Please connect using VNC or connect a monitor.")
