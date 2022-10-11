

#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.

import serial
import math
import numpy as np
#from open3d import *
import open3d as o3d

def convert(x,y,measurements):
    
    '''
    xMultiplier = 1
    yMultiplier = 1
    
    for i in range(8):
        xMultiplier = 1
        yMultiplier = 1

        
        if(i == 3 or i == 4 or i == 5):
            yMultiplier = -1
        if(i == 5 or i == 6 or i == 7):
            xMultiplier = -1
        
        if(i == 0 or i == 4):
            x.append(0)
            y.append(measurements[i] * yMultiplier)
        elif(i == 2 or i == 6):
                x.append(measurements[i] * xMultiplier)
                y.append(0)
        else:
            x.append(measurements[i] * xMultiplier * math.cos(math.pi/4))
            y.append(measurements[i] * yMultiplier * math.cos(math.pi/4)	)
    '''
    degree = 0
    for i in range(len(measurements)):
        x.append(measurements[i] * math.cos(math.pi * degree/180))
        y.append(measurements[i] * math.sin(math.pi* degree/180))
        degree += 5.625
        

s = serial.Serial('COM5',115200,timeout = 10)
#s = ser.read(100)
print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# wait for user's signal to start the program
input("Press Enter to start communication...")
# send the character 's' to MCU via UART
# This will signal MCU to start the transmission

s.write('s'.encode())
z=[]
x=[]
y=[]


measurements=[]
# recieve characters from UART of MCU
#x = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
zcount = 0
for i in range(9):
    #print(i)
    line = s.readline()                    # read one byte
    print(line.decode())

while(True):
    
    for i in range(64):
        #print(i)
        line = s.readline()                    # read one byte
        print(line.decode())
        if(line.decode() !='-1\r\n' and line.decode() !=''):
            #print("AAAA")
            measurements.append(int(line.decode().split(',')[1]))
            
            z.append(zcount)
        else:
            break
    if(line.decode() == '-1\r\n'):
       # print("BBBB")
        break
    else:
        zcount+=20

    
convert(x,y,measurements)   
# the encode() and decode() function are needed to convert string to bytes
# because pyserial library functions work with type "bytes"

#z=[0] * len(measurements)
xyz = np.zeros((3, len(x)))
xyz.shape = (len(x),3)
#xyz[0] = 1
for i in range(len(x)):
    
    xyz[i][0] = x[i]
    xyz[i][1] = y[i]
    xyz[i][2] = z[i]
    
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
o3d.visualization.draw_geometries([pcd])


#close the port
print("Closing: " + s.name)
s.close()


	
    				




