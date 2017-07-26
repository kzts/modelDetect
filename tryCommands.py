# Coding: utf-8
import os
import time

swing = './swing'

# number
#forw0 = 0
#back0 = 1
#forw1 = 2
#back1 = 3

fix = 0.3
#min0 = 0.1
#min1 = 0.1
#diff = 0.1

for min0 in [ 0.05, 0.10, 0.15, 0.20, 0.25 ]:
    for min1 in [ 0.05, 0.10, 0.15, 0.20, 0.25 ]:
        for diff in [ 0.06, 0.07, 0.08, 0.09, 0.10 ]:
#        for diff in [ 0.05, 0.06, 0.07, 0.07, 0.08 ]:
#for min0 in [ 0.05, 0.25 ]:
#    for min1 in [ 0.05, 0.25 ]:
#        for diff in [ 0.05, 0.08 ]:
            # set pressure
            back0 = min0
            back1 = min1 + fix
            forw0 = min0 + diff
            forw1 = min1
            #back0 = min0 + fix
            #back1 = min1
            #forw0 = min0
            #forw1 = min1 + diff
            # set buffuer
            command = swing + ' ' + str( forw0 ) + ' '+ str( back0 ) + ' '+ str( forw1 ) + ' '+ str( back1 )
            print command
            # execute
            os.system(command)
            # wait
            time.sleep(1)

#for n in range(0,repeat_n):
#   os.system(command)
