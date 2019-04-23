import serial
import matplotlib.pyplot as plt
from matplotlib import style
import numpy as np
import time
import sys
import argparse 

argparser = argparse.ArgumentParser('Tuning Script for yaw controller') 
argparser.add_argument('-s', help='Serial port', default='/dev/ttyUSB0')
argparser.add_argument('-n', help='Number of points to plot', default=1000)
argparser.add_argument('-P', help='Proportional Gain', default=0, type=float)
argparser.add_argument('-I', help='Integral Gain', default=0, type=float)
argparser.add_argument('-D', help='Derivative Gain', default=0, type=float) 
args = argparser.parse_args()


serial_obj = serial.Serial(args.s)
style.use('fivethirtyeight')

print('Connecting')

serial_obj.write('<change,{},{},{}>'.format(args.P, args.D, args.I).encode())

time.sleep(1)
serial_obj.write(b'<start>')

print('Starting')
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

time = []
yaw = []
finpos = []
moment = []

for i in range(args.n):
    line = serial_obj.readline().decode().strip(' ')

    if len(line) < 57:
        continue
    splitted = list(filter(lambda x: x != '', line.split(' ')))
    sys.stderr.write(line)
    floats = []
    if splitted[0] in ['start', 'change', 'f1', 'f2', 'f3']:
        continue
    for z in splitted:
        try:
            x = float(z)
            floats.append(x)
        except:
            continue
    if abs(floats[1]) > 200 or abs(floats[2]) > 1 or abs(floats[3]) > 2000:
        continue

    time.append(floats[0])
    yaw.append(180 / np.pi * floats[1])
    moment.append(floats[2])
    finpos.append(floats[3])

t = np.arange(len(time))
ax1.plot(t, yaw, label='yaw')
ax1.plot(t,moment, label='moment')
ax1.plot(t, finpos, label='fin')
plt.legend()
plt.show()
