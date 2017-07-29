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
pmin = 0.20
pfix = 0.20

for diff0 in [ 0.04, 0.08, 0.12, 0.16, 0.20 ]:
    for diff1 in [ 0.04, 0.08, 0.12, 0.16, 0.20 ]:
        if mode == '0':
            back0 = str( pmin - diff0 )
            forw0 = str( pmin + diff1 )            
            back1 = str( pmin )
            forw1 = str( pmin + pfix )
        elif mode == '1':
            back0 = str( pmin )
            forw0 = str( pmin + pfix )
            back1 = str( pmin - diff0 )
            forw1 = str( pmin + diff1 )
        else:
            back0 = str( pmin )
            forw0 = str( pmin + diff0 )
            back1 = str( pmin )
            forw1 = str( pmin + diff1 )
        command = swing + ' ' + mode + ' ' + back0 + ' ' + forw0 + ' ' + forw1 + ' '+ back1
        print command
        os.system(command)
        time.sleep(1)
