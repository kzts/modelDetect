# Coding: utf-8
import os
import time
import sys

mode  = '2';
# number: back0 = 0, forw0 = 1, forw1 = 2, back1 = 3
swing = './swing'
pmin  = 0.05
ptick = 0.02

for d0 in range( 1, 11 ):
    for d1 in range( 1, 11 ):
        diff0 = ptick* d0;
        diff1 = ptick* d1;
        back0 = str( pmin )
        forw0 = str( pmin + diff0 )
        back1 = str( pmin )
        forw1 = str( pmin + diff1 )
        command = swing + ' ' + mode + ' ' + back0 + ' ' + forw0 + ' ' + forw1 + ' '+ back1
        print command
        os.system(command)
        time.sleep(1)
#print str(d0) + ',' + str(d1)        
#print str(diff0) + ',' + str(diff1)        
