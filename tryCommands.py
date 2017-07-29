# Coding: utf-8
import os
import time
import sys

if len( sys.argv ) != 2:
    print 'input mode'
    quit()
mode  = sys.argv[1];

# number: back0 = 0, forw0 = 1, forw1 = 2, back1 = 3
swing = './swing'
pmin = 0.15
pfix = 0.15

if mode == '0':
    for diff0 in [ 0.02, 0.04, 0.06, 0.08, 0.10 ]:
        for diff1 in [ 0.02, 0.04, 0.06, 0.08, 0.10 ]:
            back0 = str( pmin - diff0 )
            forw0 = str( pmin + diff1 )            
            back1 = str( pmin )
            forw1 = str( pmin + pfix )
            command = swing + ' ' + mode + ' ' + back0 + ' ' + forw0 + ' ' + forw1 + ' '+ back1
            print command
            os.system(command)
            time.sleep(1)
elif mode == '1':
    for diff0 in [ 0.02, 0.04, 0.06, 0.08, 0.10 ]:
        for diff1 in [ 0.02, 0.04, 0.06, 0.08, 0.10 ]:
            back0 = str( pmin )
            forw0 = str( pmin + pfix )
            back1 = str( pmin - diff0 )
            forw1 = str( pmin + diff1 )
            command = swing + ' ' + mode + ' ' + back0 + ' ' + forw0 + ' ' + forw1 + ' '+ back1
            print command
            os.system(command)
            time.sleep(1)        
else:
    for diff0 in [ 0.02, 0.04, 0.06, 0.08, 0.10 ]:
        for diff1 in [ 0.02, 0.04, 0.06, 0.08, 0.10 ]:
            back0 = str( pmin )
            forw0 = str( pmin + diff0 )
            back1 = str( pmin )
            forw1 = str( pmin + diff1 )
            command = swing + ' ' + mode + ' ' + back0 + ' ' + forw0 + ' ' + forw1 + ' '+ back1
            print command
            os.system(command)
            time.sleep(1)                    

