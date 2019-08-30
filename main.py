#
# main.cc
# Release: KB3D++-3.a (03/28/08)
#
# Contact: Cesar Munoz (munoz@nianet.org)
# National Institute of Aerospace
# http://research.nianet.org/fm-at-nia/KB3D
#
# The below Python implementation was done by Phu Tran @ ATMRI
#/

from Constants import *
from Util import *
from LoS import *
from Geodesic import *
from CDR import *
from CD3D import *
from KB3D import *


if __name__ == "__main__":
  
    D = 5    # 5 nautical miles
    H = 1000 # 1000 ft
    T = 300  # 5 minutes

    # Create a CDR object
    '''
    ownship : lon [deg] lat[deg] alt[feet] trk[deg] gs[knots] vs [ft/min]
               10,    -0.383,     11500,     96,      253,     -500,
    traffic: lon [deg] lat[deg] alt[feet] trk[deg] gs[knots] vs [ft/min]
               10,         0,     10000,    287,      316,      200)
    '''
    cdr = CDR( D, H, T, 10, -0.383, 11500, 96, 253, -500,  10, 0, 10000, 287, 316, 200, True )
    
    print("Distance:", cdr.distance, "[nm]") 
    print("Course:", cdr.course, "[deg]")    
    
    # Check if there is a conflict
    if cdr.detection()  :
    
        # Check if there is a violation
        
        if cdr.violation() :
            print("Aircraft are in violation")
            print("Time of entry:", cdr.t_in, "[sec]")
            print("Time of exit:", cdr.t_out, "[sec]")        
        
        else :
            print("Time to loss of separation:", cdr.time2los, "[sec]")
            print("Time to loss of horizontal separation:", cdr.time2lhs, "[sec]")
            print("Time to loss of vertical separation:", cdr.time2lvs, "[sec]")
            print("Duration of conflict:", cdr.duration, "[sec]")
        
        # Call resolution algorithm
        
        cdr.resolution()
        
        if cdr.newtrk != NaR :
            print("Track Only =", cdr.newtrk, "[deg]")
        
        if cdr.newgs != NaR :
            print("Ground Speed Only =", cdr.newgs, "[knots]")
        
        if cdr.opttrk != NaR :
            print("Optimal Track =", cdr.opttrk, "[deg]")
        
        if cdr.optgs != NaR :
            print("Optimal Ground Speed =", cdr.optgs, "[knots]")
        
        if cdr.newvs != NaR :
            print("Vertical Speed Only =", cdr.newvs, "[ft/min]")
    
    else :
        print("No predicted conflict in", T, "[sec] (filter:", cdr.get_detection_filter(), "[sec])")
    
    del cdr