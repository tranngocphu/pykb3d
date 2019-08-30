#
# KB3D.cpp
# Release: KB3D++-3.a (03/28/08)
#
# Contact: Cesar Munoz (munoz@nianet.org)
# National Institute of Aerospace
# http://research.nianet.org/fm-at-nia/KB3D
#
# KB3D is an algorithm for 3-D conflict #resolution# that provides
# avoidance maneuvers for the ownship 
#
# Unit Convention
# ---------------
# - Units of distance are denoted [d]
# - Units of time are denoted     [t]
# - Units of speed are denoted    [d/t]
# - Units of turn are radians     [rad] 
#
# REMARK: X points to East, Y points to North. Angles are in 
#         True North/clockwise convention.
#
# Naming Convention
# -----------------
# The traffic aircraft is supposed to be fixed at the origin of the coordinate
# system. 
# D           : Horizontal separation [d]
# H           : Vertical separation [d]
# T           : Lookahead time [t]
# vox,voy,voz : Ownship velocity vector  [d/t,d/t,d/t]
# vix,viy,viz : Traffic velocity vector [d/t,d/t,d/t]
#
# Functions
# ---------
# - ground_speed, vertical, track : Independent resolutions
# - kb3d_vertical   : Coordinated vertical maneuver (vertical speed)
# - kb3d_horizontal : Coordinated horizontal maneuvers (ground speed, track)
#
# Output class variables
# ----------------------
# trk: Track [rad]
# gs : Ground speed   [d/t]
# vs : Vertical speed [d/t]
#
# The below Python implementation was done by Phu Tran @ ATMRI. Comments are from original C++ code.
#

from Constants import *
from Util import *
from CD3D import *



# Support functions 

# Time of closest approach (horizontal plane)
def tau( sx, sy, vx, vy) : 
    a = sq(vx) + sq(vy)
    if a != 0 :
        return -(sx*vx + sy*vy) / a 
    return NaR


# Is tau positive ?
def tau_pos( sx, sy, vx, vy) :
    return sx*vx + sy*vy <= -TAU_MIN


# Epsilon
def eps_line( sx, sy, vx, vy) :
    return sign( sx*vx + sy*vy ) / sign( sx*vy - sy*vx )


def break_symm( sx, sy, sz) :
    if sx < 0 or ( sx == 0 and sy < 0 ) :
        return 1
    return -1


def contact_time( sx, sy, qx, qy, vx, vy ) :
    d = vx*(qx-sx) + vy*(qy-sy)
    if d != 0 :
        return ( sq(qx-sx) + sq(qy-sy) ) / d
    if qx == sx and qy == sy :
        return 0
    return -1



class KB3D :


    def __init__( self, d, h, t ) :
        self.D = d
        self.H = h
        self.T = t
        self.cd3d = CD3D( self.D, self.H, self.T )
        self.vs = NaR
        self.gs = NaR
        self.trk = NaR
        self.opt_gs = NaR
        self.opt_trk = NaR
        self.vx = 0
        self.vy = 0
        self.kvx = 0
        self.kvy = 0
        self.ovx = 0
        self.ovy = 0


    def clear_KB3D( self ) :
        del self.cd3d


    def set_D( self, d ) :
        self.cd3d.set_D( d )
        self.D = d


    def get_D( self ) :
        return self.D

    
    def set_H( self, h ) :
        self.cd3d.set_H( h )
        self.H = h


    def get_H( self ) :
        return self.H


    def set_T( self, t ) :
        self.cd3d.set_T( t )
        self.T = t


    def get_T( self ) :
        return self.T


    
    # General Precondition for KB3D
    #
    # - Aircraft are in predicted conflict
    # - Aircraft are not in violation
    # - Aircraft have a positive ground speed
    #
    #

    def precondition( self, sx, sy, sz, vox, voy, voz, vix, viy, viz ) :

        cond1 = not self.cd3d.violation( sx, sy, sz )
        cond2 = self.cd3d.cd3d( sx, sy, sz, vox - vix, voy - viy, voz - viz )
        cond3 = sq(vox) + sq(voy) > 0
        cond4 = sq(vix) + sq(viy) > 0
        return cond1 and cond2 and cond3 and cond4


    # Coordination strategies
    
    def vertical_coordination( self, sx, sy, sz, pz ) :
        epsilon = 1        
        if pz == 0 and sz == 0 :
            epsilon = break_symm( sx, sy, sz )
        elif pz == 0 : 
            epsilon = sign(sz)
        else :
            epsilon = sign(pz)
        return epsilon


    def horizontal_coordination( self, sx, sy, vx, vy) :
        return sign( sy*vx - sx*vy )
  

    def delta( self, sx, sy, vx, vy ) :
        return sq(self.D) * ( sq(vx) + sq(vy) ) - sq( sx*vy - sy*vx )
    

    def theta( self, sx, sy, vx, vy, eps ) :
        v = sq(vx) + sq(vy)
        if v == 0 :
            return 0 # THIS CASE SHOULD NEVER HAPPEN
        d = self.delta( sx, sy, vx, vy )
        return ( -sx*vx - sy*vy + eps*sqrt(d) ) / v    


    def vertical_theta1( self, sx, sy, sz, vx, vy, viz, eps ) :
        t = self.theta( sx, sy, vx, vy, -1 )
        if t == 0 :
            return NaR # THIS CASE SHOULD NEVER HAPPEN
        return viz + ( eps * self.H - sz ) / t
    

    def vertical_theta2( self, sx, sy, sz, vx, vy, viz ) : 
        t = self.theta( sx, sy, vx, vy, 1 )
        if t == 0 :
            return NaR # THIS CASE SHOULD NEVER HAPPEN
        return viz + ( sign(sz) * self.H - sz ) / t
    
    '''
    # vertical: Independent vertical speed only maneuver
    #
    # Let vs = vertical(...) in
    # vs != NaR ==>
    #   (vox,voy,vs) is an independent ground speed only solution for the ownship.
    #
    '''

    def vertical( self, sx, sy, sz, vox, voy, voz, vix, viy, viz, epsilon ) :
        self.vs = NaR
        self.vx = vox - vix
        self.vy = voy - viy
        if sq(self.vx) + sq(self.vy) == 0 :
            self.vs = viz
        elif epsilon*sz < self.H and sq(sx) + sq(sy) > sq(self.D) :
            self.vs = self.vertical_theta1( sx, sy, sz, self.vx, self.vy, viz, epsilon )        
        elif epsilon*sz >= self.H : 
            self.vs =  self.vertical_theta2( sx, sy, sz, self.vx, self.vy, viz )        
        return self.vs
    

    def ground_speed_k( self, sx, sy, vox, voy, vix, viy, epsilon ) :    
        k = 0
        a = sq(self.D) * ( sq(vox) + sq(voy) ) - sq(sx*voy - sy*vox)
        b = 2*( (sx*voy - sy*vox) * (sx*viy - sy*vix) - sq(self.D) * (vox*vix + voy*viy) )
        c = sq(self.D) * ( sq(vix) + sq(viy) ) - sq(sx*viy - sy*vix)
        if a == 0  and ( c*b >= 0 or not tau_pos( sx, sy, -c/b*vox - vix, -c/b*voy - viy ) ) : 
            k = 0
        elif a == 0 :
            k = -c/b
        elif discr(a,b,c) >= 0 :
            k = root( a, b, c, epsilon ) 
            if k <= 0 or not tau_pos( sx, sy, k*vox - vix, k*voy - viy ) :
                k = 0        
        return k

    
    '''
    * ground_speed: Independent ground speed only maneuver
    *
    * Let gs = ground_speed(...) in
    * gs != NaR ==>
    *   gs = || kvx,kvy ||, where 
    *   kvx = k*vox, kvy = k*voy
    *   (kvx,kvy,voz) is an independent ground speed only solution 
    *   for the ownship.
    *
    * OUTPUTS: kvx,kvy 
    * 
    '''

    def ground_speed( self, sx, sy, vox, voy, vix, viy, epsilon ) :
        k = self.ground_speed_k( sx, sy, vox, voy, vix, viy, epsilon )
        self.kvx = 0 
        self.kvy = 0
        if k > 0 and eps_line( sx, sy, k*vox - vix, k*voy - viy ) == epsilon :
            self.kvx = k*vox
            self.kvy = k*voy
        else :
            k = self.ground_speed_k( sx, sy, vox, voy, vix, viy, -epsilon )
            if k > 0 and eps_line( sx, sy, k*vox - vix, k*voy - viy ) == epsilon :
                self.kvx = k*vox
                self.kvy = k*voy                    
        if self.kvx != 0 or self.kvy != 0 : 
            return sqrt( sq(self.kvx) + sq(self.kvy) )
        return NaR
    

    def track_vx_vy( self, sx, sy, vox, voy, vix, viy, epsilon ) :
        self.vx = 0 
        self.vy = 0
        s2   = sq(sx) + sq(sy)
        v2   = sq(vox) + sq(voy)
        R    = self.D / sqrt( s2 - sq(self.D) )
        sxy  = sx - epsilon*R*sy
        syx  = sy + epsilon*R*sx
        viyx = viy*sxy - vix*syx
        a    = sq(s2) / ( s2 - sq(self.D) )
        b    = 2*syx*viyx
        c    = sq(viyx) - sq(sxy)*v2
        if sxy == 0 and ( syx == 0 or v2 < sq(vix) ) :
            return
        if sxy == 0 :
            self.vy = sign(voy) * sqrt( v2 - sq(vix) )
            if tau_pos( sx, sy , 0, self.vy-viy ) :
                self.vx = vix
                self.vy = self.vy
            elif tau_pos( sx, sy, 0, -self.vy-viy ) :
                self.vx = vix
                self.vy = -self.vy             
            else :
                return
        elif discr(a,b,c) >= 0 :
            vx1 = root( a, b, c, 1 )
            vy1 = ( viyx + syx*vx1 ) / sxy
            vx2 = root(a,b,c,-1)
            vy2 = (viyx + syx*vx2) / sxy
            tp1 = tau_pos( sx, sy, vx1-vix, vy1-viy )
            tp2 = tau_pos( sx, sy, vx2-vix, vy2-viy )
            if tp1 and ( not tp2 or vx1*vox+vy1*voy > vx2*vox+vy2*voy ) :
                self.vx = vx1
                self.vy = vy1 
            elif tp2 :
                self.vx = vx2
                self.vy = vy2 
            else :
                return

    '''
    /* track: Independent track only maneuver
    *
    * Let trk = heanding(...) in
    * trk != NaR ==>
    *   trk = atan2(vx,vy) and 
    *   (vx,vy,voz) is an independent track only solution for the ownship.
    *
    * REMARK: trk is in True North/clockwise convention.
    *
    * OUTPUTS: vx,vy
    */
    '''

    def track( self, sx, sy, vox, voy, vix, viy, epsilon ) :
        self.track_vx_vy( sx, sy, vox, voy, vix, viy, epsilon )
        if self.vx != 0 or self.vy != 0 :
            return atan2_safe( self.vx, self.vy )
        return NaR
        

    def alpha( self, sx, sy) :
        if sx != 0 or sy != 0 :
            return sq(self.D) / ( sq(sx) + sq(sy) )
        return 0


    def beta( self, sx, sy ) :
        if sx != 0 or sy != 0 :
            return self.D * sqrt_safe( sq(sx) + sq(sy) - sq(self.D) ) / ( sq(sx) + sq(sy) )
        return 0


    def Q( self, sx, sy, epsilon ) :
        return self.alpha(sx,sy)*sx + epsilon*self.beta(sx,sy)*sy
    

    def optimal_vx_vy( self, sx, sy, vox, voy, vix, viy, epsilon ) :
        self.ovx = 0 
        self.ovy = 0
        vx  = vox-vix    
        vy  = voy-viy
        qpx = self.Q( sx, sy, epsilon )
        qpy = self.Q( sy, sx, -epsilon )
        tpq = contact_time( sx, sy, qpx, qpy, vx, vy )
        if tpq > 0 :
            self.ovx = (qpx-sx) / tpq + vix
            self.ovy = (qpy-sy) / tpq + viy
        elif tpq == 0 :
            self.ovx = -sy*(sx*vy-vx*sy) + vix
            self.ovy =  sx*(sx*vy-vx*sy) + viy
    
    
    '''
    /* optimal: Independent optimal track and ground speed only maneuver
    *
    * optimal(...) computes opt_trk and opt_gs, where
    * opt_trk != NaR and opt_gs != NaR ==>
    *   opt_trk = atan2(ovx,ovy) and opt_gs = || ovx,ovy || and
    *   (ovx,ovy,voz) is an optimal combined track and ground speed resolution
    *   for the the ownship.
    *
    * REMARK: opt_trk is in True North/clockwise convention.
    *
    * OUTPUTS: opt_trk,opt_gs,ovx,ovy
    */
    '''

    def optimal( self, sx, sy, vox, voy, vix, viy, epsilon) :
        self.opt_trk = NaR
        self.opt_gs = NaR
        self.optimal_vx_vy( sx, sy, vox, voy, vix, viy, epsilon )
        if ( self.ovx != 0 or self.ovy != 0 ) and eps_line( sx, sy, self.ovx-vix, self.ovy-viy) == epsilon :
            self.opt_trk = atan2_safe( self.ovx, self.ovy )
            self.opt_gs  = sqrt( sq(self.ovx) + sq(self.ovy) )
            return        
        self.optimal_vx_vy( sx, sy, vox, voy, vix, viy, -epsilon)
        if ( self.ovx != 0 or self.ovy != 0 ) and eps_line( sx, sy, self.ovx-vix, self.ovy-viy ) == epsilon :
            self.opt_trk = atan2_safe( self.ovx, self.ovy )
            self.opt_gs  = sqrt( sq(self.ovx) + sq(self.ovy) )
            return        
        self.ovx = 0 
        self.ovy = 0
    
    '''
    /* kb3d_vertical: Coordinated vertical maneuver.
    *
    * PRECONDITION:
    *   precondition(...) 
    *
    * POSTCONDITION:
    *   vs != NaR ==>
    *     (vox,voy,vs) is a coordinated vertical only resolution 
    *     for the ownship.
    *
    * OUTPUTS: vs
    *
    */
    '''

    def kb3d_vertical( self, sx, sy, sz, vox, voy, voz, vix, viy, viz) :
        self.vs = NaR
        if self.precondition( sx, sy, sz, vox, voy, voz, vix, viy, viz ) :
            pz = sz + self.cd3d.time2los*(voz-viz)
            self.vs = self.vertical( sx, sy, sz, vox, voy, voz, vix, viy, viz, self.vertical_coordination( sx, sy, sz, pz) )
    
    

    '''    
    /* kb3d_horizontal: Coordinated horizontal maneuvers.
    *
    * PRECONDITION:
    *   precondition(...) &&
    *   sq(sx)+sq(sy) > sq(D)
    *
    * POSTCONDITION:
    *   gs != NaR ==>
    *     gs yields a coordinated ground speed only solution for the ownship.
    *   trk != NaR ==>
    *     trk yields a coordinated track only solution for the ownship.
    *   opt_trk != NaR and opt_gs != NaR ==>
    *     opt_trk and opt_gs yield a coordinated optimal combined track and ground
    *     speed solution for the ownship.
    *
    * REMARK: trk and opt_tr are in True North/clockwise convention.
    *
    * OUTPUTS: trk,gs,opt_gs,opt_trk
    *
    */
    '''


    def kb3d_horizontal( self, sx, sy, sz, vox, voy, voz, vix, viy, viz) :
        self.gs      = NaR 
        self.trk     = NaR
        self.opt_trk = NaR
        self.opt_gs  = NaR    
        if self.precondition( sx, sy, sz, vox, voy, voz, vix, viy, viz ) and sq(sx)+sq(sy) > sq(self.D) :
            epsilon = self.horizontal_coordination( sx, sy, vox-vix, voy-viy )
            self.gs  = self.ground_speed( sx, sy, vox, voy, vix, viy, epsilon )
            self.trk = self.track( sx, sy, vox, voy, vix, viy, epsilon )
            self.optimal( sx, sy, vox, voy, vix, viy, epsilon )    


    '''
    /* kb3d: Coordinated horizontal and vertical maneuvers 
    *
    * PRECONDITION:
    *   precondition(...) &&
    *   sq(sx)+sq(sy) > sq(D)
    *
    * POSTCONDITION:
    *   vs != NaR ==>
    *     (vox,voy,vs) is a coordinated vertical only resolution 
    *     for the ownship.
    *   gs != NaR ==>
    *     gs yields a coordinated ground speed only solution for the ownship.
    *   trk != NaR ==>
    *     trk yields a coordinated track only solution for the ownship.
    *   opt_trk != NaR and opt_gs != NaR ==>
    *     opt_trk and opt_gs yield a coordinated optimal combined track and ground
    *     speed solution for the ownship.
    *
    * REMARK: trk and opt_tr are in True North/clockwise convention.
    *
    * OUTPUTS: trk,gs,opt_trk,opt_gs,vs
    *
    */
    '''

    def kb3d( self, sx, sy, sz, vox, voy, voz, vix, viy, viz) :
        self.kb3d_vertical( sx, sy, sz, vox, voy, voz, vix, viy, viz )
        self.kb3d_horizontal( sx, sy, sz, vox, voy, voz, vix, viy, viz )