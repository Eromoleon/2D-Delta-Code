import os
# True if a display (real or virtual like VNC) is connected.
disp = False 
try:
	d = os.environ['DISPLAY']
except:
	print("No display connected...")
else:
	disp = True
