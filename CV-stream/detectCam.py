import subprocess

c = subprocess.check_output(["vcgencmd","get_camera"])
if c==b'supported=1 detected=0\n':
    print('No camera detected')
else:
    print('Camera detected')
print(c.decode())
#int(camdet.strip()[-1]) #-- Removes the final CR character and gets only the "0" or "1" from detected status

#if (c):
#    print ("Camera detected")
#else:
#    print ("not detected")
