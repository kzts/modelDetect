# Coding: utf-8
import os
import time
import sys

mode  = '2';
# number: back0 = 0, forw0 = 1, forw1 = 2, back1 = 3
swing = './swing'
pmin  = 0.05
ptick = 0.02
poffs = 0.05

for d0 in range( 1, 9 ):
    for d1 in range( 1, 9 ):
        diff0 = ptick* d0 + poffs;
        diff1 = ptick* d1 + poffs;
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
