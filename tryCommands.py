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
pmin = 0.05
pfix = 0.30
d_tick = 0.02;

if mode == '0':
    smode = '0'
    for d in range( 1, 11 ):
        diff  = d_tick* d + pmin 
        back0 = str( pmin  )
        forw0 = str( pmin + diff )            
        back1 = str( pmin )
        forw1 = str( pmin + pfix )
        command = swing + ' ' + smode + ' ' + back0 + ' ' + forw0 + ' ' + forw1 + ' '+ back1
        print command
        os.system(command)
        time.sleep(1)
elif mode == '1':
    smode = '0'
    for d in range( 1, 11 ):
        diff  = d_tick* d + pmin 
        back0 = str( pmin  )
        forw0 = str( pmin + diff )            
        back1 = str( pmin + pfix)
        forw1 = str( pmin )
        command = swing + ' ' + smode + ' ' + back0 + ' ' + forw0 + ' ' + forw1 + ' '+ back1
        print command
        os.system(command)
        time.sleep(1)
if mode == '2':
    smode = '1'
    for d in range( 1, 11 ):
        diff  = d_tick* d + pmin
        back0 = str( pmin  )
        forw0 = str( pmin + pfix )            
        back1 = str( pmin )
        forw1 = str( pmin + diff )
        command = swing + ' ' + smode + ' ' + back0 + ' ' + forw0 + ' ' + forw1 + ' '+ back1
        print command
        os.system(command)
        time.sleep(1)
elif mode == '3':
    smode = '1'
    for d in range( 1, 11 ):
        diff  = d_tick* d + pmin
        back0 = str( pmin + pfix )
        forw0 = str( pmin  )            
        back1 = str( pmin )
        forw1 = str( pmin + diff )
        command = swing + ' ' + smode + ' ' + back0 + ' ' + forw0 + ' ' + forw1 + ' '+ back1
        print command
        os.system(command)
        time.sleep(1)
elif mode == '4':
    smode = '2'
    for diff0 in [ 0.04, 0.08, 0.12, 0.16, 0.20 ]:
        for diff1 in [ 0.04, 0.08, 0.12, 0.16, 0.20 ]:
            back0 = str( pmin )
            forw0 = str( pmin + diff0 )
            back1 = str( pmin )
            forw1 = str( pmin + diff1 )
            command = swing + ' ' + smode + ' ' + back0 + ' ' + forw0 + ' ' + forw1 + ' '+ back1
            print command
            os.system(command)
            time.sleep(1)
