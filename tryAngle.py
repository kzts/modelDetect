# Coding: utf-8
import os
import time
import random

control = './controlAngle'
p_gain = 0.000025 
i_gain = 0.0000002 
d_gain = 0 
command_pre = control + ' ' + str( p_gain ) + ' '+ str( i_gain ) + ' '+ str( d_gain );

min0 = 1100
max0 = 2500
min1 = 500
max1 = 2200

#1: 961-2718, 
#2: 379-2491
# ./controlAngle 0.000025 0.0000002 0 2000 1000
for n in range(0,50):
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


