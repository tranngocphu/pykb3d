 #
 # CDR.cpp
 # Release: KB3D++-3.a (03/28/08)
 #
 # Contact: Cesar Munoz (munoz@nianet.org)
 # National Institute of Aerospace
 # http://research.nianet.org/fm-at-nia/KB3D
 #
 # Simple interface to CD3D and KB3D. State information for each aircraft is
 # given as:
 # lat [deg], lon [deg], alt [feet], trk [deg], gs [knots], vs [feet/min]
 #
 # D [nm]   : Horizontal separation  
 # H [feet] : Vertical separation
 # T [sec]  : Lookahead time
 #
 # REMARK: East longitudes are positive.
 #         Degrees are in True North/clockwise convention.
 #
 # BASIC USAGE:
 #   // Create a CDR object
 #   CDR# cdr = new CDR(D,H,T,
 #                      lat_o,lon_o,alt_o,trk_o,gs_o,vs_o,
 #                      lat_i,lon_i,alt_i,trk_i,gs_i,vs_i);
 #
 #  // Check if there is a conflict
 #  if (cdr->detection()) {
 #    // Check if there is a violation
 #     if (!cdr->violation()) {
 #       // Call resolution algorithm
 #       cdr->resolution();
 #       ...
 #     }
 #  ...
 # }
 # 
 #
 # The below Python implementation was done by Phu Tran @ ATMRI
 #

import math
from math import sin, cos, fabs, asin
from Constants import *
from Util import *
from LoS import *
from Geodesic import *
from CD3D import *
from KB3D import *



class CDR :


    def __init__ (  self, D, H, T, x_o, y_o, alt_o, trk_o, gs_o, vs_o, x_i, y_i, alt_i, trk_i, gs_i, vs_i, gxy ) :

        self.d = nm2m(D)        
        self.h = ft2m(H)
        self.t = T

        self.cd = CD3D(self.d, self.t, self.h)
        self.cr = KB3D(self.d, self.h, self.t)

        if gxy :

            geo = Geodesic()
            geo.geo2xy(x_o,y_o,x_i,y_i)
            self.sx = geo.sx
            self.sy = geo.sy
            self.distance = rad2deg(geo.gcd) * 60.0
            self.course = rad2deg(geo.tc) 
            del geo

        else :

            self.sx = nm2m( x_o - x_i )
            self.sy = nm2m( y_o - y_i )
            self.distance = sqrt_safe( sq( x_i - x_o ) + sq( y_i - y_o ) )
            self.course = rad2deg( atan2_safe( x_i - x_o, y_i - y_o ) )

        self.sz = ft2m( alt_o - alt_i )
        
        gs   = knots2msec(gs_o)
        trk  = deg2rad(trk_o)
        self.vox = gs2vx(gs,trk)
        self.voy = gs2vy(gs,trk)
        self.voz = ftmin2msec(vs_o)
        
        gs  = knots2msec(gs_i)
        trk = deg2rad(trk_i)
        self.vix = gs2vx(gs,trk)
        self.viy = gs2vy(gs,trk)
        self.viz = ftmin2msec(vs_i)

        self.time2los = 0
        self.t_in = 0
        self.t_out = 0
        self.duration = 0
        self.filter = 1
        self.recovery = 0
        
        self.newtrk = NaR
        self.newgs = NaR
        self.newvs = NaR
        self.opttrk = NaR
        self.optgs = NaR

    
    def clear_CDR( self ) :
        del self.cd, self.cr
        

    def violation( self ) :
        self.cd.set_D(self.d)
        self.cd.set_H(self.h)
        self.cd.set_T(self.t)
        return self.cd.violation( self.sx, self.sy, self.sz )

    
    def detection( self ) :
        self.cd.set_D(self.d)
        self.cd.set_H(self.h)
        self.cd.set_T(self.t)
        self.cd.set_filter(self.filter)
        self.cd.cd3d(self.sx, self.sy, self.sz, self.vox - self.vix, self.voy - self.viy, self.voz - self.viz)
        self.time2los = self.cd.time2los
        self.time2lhs = self.cd.time2lhs
        self.time2lvs = self.cd.time2lvs
        self.duration = self.cd.duration
        self.t_in = self.cd.t_in
        self.t_out = self.cd.t_out
        return self.cd.conflict


    def resolution( self ) :
        cdnow = CD3D( self.d, self.h, self.t )
        self.cr.set_D(self.d)
        self.cr.set_H(self.h)
        self.cr.set_T(self.t)
        self.recovery = 0

        if cdnow.cd3d( self.sx, self.sy, self.sz, self.vox - self.vix, self.voy - self.viy, self.voz - self.viz ) :
            
            if not cdnow.violation( self.sx, self.sy, self.sz ) : # aircraft are separated
                self.cr.kb3d( self.sx, self.sy, self.sz, self.vox, self.voy, self.voz, self.vix, self.viy, self.viz )                
                
                if self.cr.trk != NaR : 
                    self.recovery = 1
                    self.newtrk = rad2deg(self.cr.trk)                
                
                if self.cr.gs != NaR :
                    self.recovery = 1
                    self.newgs = msec2knots(self.cr.gs)                
                
                if self.cr.opt_trk != NaR :
                    self.recovery = 1
                    self.opttrk = rad2deg(self.cr.opt_trk)
                    self.optgs = msec2knots(self.cr.opt_gs)
                
                if self.cr.vs != NaR :
                    self.recovery = 1
                    self.newvs = msec2ftmin(self.cr.vs)                
            
            else : # aircraft are in violation
                self.recovery = -1
                self.newvs = msec2ftmin( vertical_recovery( self.sx, self.sy, self.sz, self.voz, self.viz, self.h, fabs(cdnow.t_in) ) )


    def set_DHT( self, D, H, T ) :
        self.d = nm2m(D)
        self.h = ft2m(H)
        self.t = T


    def get_D( self ) :
        return m2nm( self.d )


    def get_H( self ) :
        return m2ft( self.h )


    def get_T( self ) :
        return self.t

    
    def set_detection_filter( self, f ) :
        self.filter = f


    def get_detection_filter( self ) :
        return self.filter