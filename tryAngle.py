# Coding: utf-8
import os
import time
import random

control = './controlAngle'
p_gain = 0.00001 
i_gain = 0.0000001 
d_gain = 0 
command_pre = control + ' ' + str( p_gain ) + ' '+ str( i_gain ) + ' '+ str( d_gain );

min0 = 1300
max0 = 2300
min1 = 800
max1 = 1800

#1: 961-2718, 
#2: 379-2491
# ./controlAngle 0.000025 0.0000002 0 2000 1000
for n in range(0,100):
    # angles
    angle0 = random.randint( min0, max0 )
    angle1 = random.randint( min1, max1 )
    # set buffuer
    command = command_pre + ' '+ str( angle0 ) + ' '+ str( angle1 )
    print command
    # execute
    os.system(command)
    # wait
    time.sleep(1)


