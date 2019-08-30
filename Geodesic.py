#
# Geodesic.cpp
# Release: KB3D++-3.a (03/28/08)
#
# Contact: Cesar Munoz (munoz@nianet.org)
# National Institute of Aerospace
# http://research.nianet.org/fm-at-nia/KB3D
#
# This file contains the transformations of coordinate systems that
# are necessary to run KB3D.
#
# REFERENCE: [Will] Ed Williams, Aviation Formulary V1.42
#            http://williams.best.vwh.net/avform.htm
#            
# DISCLAIMER :These approximations fail in the vicinity of either pole 
#             and at large distances. The fractional errors are of order
#             (distance/R)^2. See [Will] 
#
# REMARK : North latitudes are positive.
#          East longitudes are positive.
#          X points to East, Y points to North. 
#
# Unit Conventions
# ----------------
# - lat,lan,lon in radians [rad]
# - x,y in meters [m]
#
# Functions
# ---------
#   gc_dist : Great circle distance
#   tc      : Initial true course
#   geo2xy  : Conversion from geodesic coordinates to Cartesian coordinates 
# 
# The below Python implementation was done by Phu Tran @ ATMRI
#

from math import sin, cos
from Constants import Flat, WGS84
from Util import *

# Great cricle distance [rad]
def gc_dist( lat1, lon1, lat2, lon2 ) :
    return 2 * asin_safe( sqrt_safe( sq( sin( (lat1-lat2) / 2.0 ) ) + cos(lat1)*cos(lat2)*sq( sin( (lon1-lon2) / 2.0  ) ) ) )


# Initial true course [rad]
def true_course( lat1, lon1, lat2, lon2 ) :
    return atan2_safe( sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2) * cos(lon1-lon2) )


class Geodesic : 

    def __init__ ( self ) :

        self.sx = 0
        self.sy = 0
        self.gcd = 0
        self.tc = 0


    # geo2xy:  Converts from geodesic coordinates to cartesian coordinates 
    #          assuming flat earth
    # PRECONDITION: 
    #   latdeg_o,londeg_o: Ownship  position
    #    ([deg],[deg])
    #   latdeg_i,londeg_i: Traffic position
    #    ([deg],[deg])
    #
    # POSTCONDITION: 
    #   (sx,sy) is the relative position of the ownship wrt traffic [m,m]
    #    gcd : Great circle distance [rad]
    #    tc  : Initial true course [rad]
    #
    # OUTPUTS: sx,sy,gcd,tc
    #
    def geo2xy( self, latdeg_o, londeg_o, latdeg_i, londeg_i ) :
        
        # lat_o
        # lon_o
        # lat_i
        # lon_i
        e2 = Flat * (2 - Flat)

        lat_o = deg2rad(latdeg_o)
        lat_i = deg2rad(latdeg_i)
        lon_o = deg2rad(londeg_o)
        lon_i = deg2rad(londeg_i)

        dlat = lat_o - lat_i
        dlon = lon_o - lon_i
        r32 = 1 - e2 * sq( sin(lat_i) )
        R1 = WGS84 * (1 - e2) / sqrt_safe( r32 * r32 * r32 )
        R2 = WGS84 / sqrt_safe( 1 - e2 * sq( sin( lat_i ) ) )

        self.sy  = R1 * dlat
        self.sx  = R2 * cos(lat_i) * dlon
        self.gcd = gc_dist( lat_o, lon_o, lat_i, lon_i )
        self.tc = true_course( lat_o, lon_o, lat_i, lon_i )