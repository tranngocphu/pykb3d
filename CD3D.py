# 
# CD3D.cpp
# Release: KB3D++-3.a (03/28/08)
#
# Contact: Cesar Munoz (munoz@nianet.org)
# National Institute of Aerospace
# http://research.nianet.org/fm-at-nia/KB3D
#
# CD3D is an algorithm for 3-D conflict#detection#.
#
# Unit Convention
# ---------------
# - Units of distance are denoted [d]
# - Units of time are denoted     [d]
# - Units of speed are denoted    [d/t]
#
# REMARK: X points to East, Y points to North. 
#
# Naming Convention
# -----------------
#   The traffic aircraft is fixed at the origin of the coordinate system.
# 
#   D         : Horizontal separation [d]
#   H         : Vertical separation [d]
#   T         : Lookahead time [t]
#   sx,sy,sz  : Relative position of the ownship [d,d,d]
#   vx,vy,vz  : Relative velocity vector of the ownship [d/t,d/t,d/t]
# 
# Functions
# ---------
# - cd3d : Conflict detection with calculation of conflict interval 
# 
# The below Python implementation is done by Phu Tran @ ATMRI. Comments are from original C++ code
# 


import numpy as np
import math



class CD3D :

    
    def __init__( self, d, h, t ) :
        self.D = d
        self.H = h
        self.T = t
        self.time2los = 0
        self.time2lvs = 0
        self.time2lhs = 0
        self.t_in = 0
        self.t_out = 0
        self.duration = 0
        self.conflict = False
        self.filter = 0

    
    def get_filter ( self ) :
        return self.filter
    
    
    def set_filter ( self, t ) :
        self.filter = t

    
    def set_D ( self, d ) :
        self.D = d

    
    def get_D ( self ) :
        return self.D


    def set_H ( self, h ) :
        self.H = h


    def get_H ( self ) :
        return self.H

    
    def set_T ( self, t ) :
        self.T = t


    def get_T ( self ) :
        return self.T

    
    # Check if aircraft are in violation at time 0
    def violation ( self, sx, sy, sz ) :
        return sx**2 + sy**2 < self.D**2 and sz**2 < self.H**2

    
    # cd3d: Conflict detection with conflict interval
    #
    # Let conflict = cd3d(sx,sy,sz,vx,vy,vz) in
    #   conflict ==> 
    #     time2los is the time to conflict
    #     duration is the duration of the conflict
    #
    #   violation ==>
    #     time2los = 0
    #     duration is the remaining time to exit the violation.
    #
    #   conflicts are reported only if duration > filter and time2los <= T
    #
    # OUTPUTS: time2los,duration,conflict
    #

    def cd3d ( self, sx, sy, sz, vx, vy, vz ) :

        # d
        # a
        # b
        # t1
        # t2
        # theta1
        # theta2
        
        self.conflict = False
        self.t_in = 0
        self.t_out = 0

        if vx == 0 and vy == 0 and vx**2 + vy**2 < self.D**2 :
            # There's no horizontal movement
            
            conflict = sz**2 < self.H**2 or ( vz != 0 and vz*sz <= 0 and -self.H < np.sign(vz)*(self.T*vz + sz) )

            if conflict :
                
                if vz != 0 :
                    
                    self.t_in     = (  np.sign(sz)*self.H - sz ) / vz 
                    self.t_out    = ( -np.sign(sz)*self.H - sz ) / vz 
                    time2lvs = t_in
                
                else :

                    self.t_in = 0
                    self.t_out = T

        else :
            # vertical conflict in the future
            
            d = 2 * sx * vx * sy * vy + D**2 * ( vx**2 + vy**2 ) - ( sx**2 * vy**2 + sy**2 * vx**2 )

            if d > 0 :

                a = vx**2 + vy**2
                b = sx*vx + sy*vy
                theta1 = ( -b - math.sqrt(d) ) / a # first intersection with D
                theta2 = ( -b + math.sqrt(d) ) / a # second intersection with D
                time2lhs = theta1

                # theta1 <= theta2 

                if vz == 0 :
                    
                    # horizontal movement only
                    self.t_in  = theta1
                    self.t_out = theta2
                    conflict = sz**2 < self.H**2

                else :
                    
                    # general case
                    t1 = ( -np.sign(vz)*self.H - sz ) / vz
                    t2 = (  np.sign(vz)*self.H - sz ) / vz
                    time2lvs = t1

                    # t1 < t2

                    self.t_in     = max(theta1, t1)
                    self.t_out    = min(theta2, t2)
                    conflict = theta1 < t2 and t1 < theta2

        self.time2los = max( t_in, 0 )            
        self.time2lhs = max( time2lhs, 0)
        self.time2lvs = max( time2lvs, 0)
        self.duration = self.t_out - self.time2los
        
        conflict &= self.t_in <= self.T and self.t_out > 0 and self.duration > self.filter
        
        self.conflict = conflict
        
        return conflict
