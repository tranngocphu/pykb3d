#
# Util.cpp
# Release: KB3D++-3.a (03/28/08)
#
# Contact: Cesar Munoz (munoz@nianet.org)
# National Institute of Aerospace
# http://research.nianet.org/fm-at-nia/KB3D
#
# The below Python imlementation was done by Phu Tran @ ATMRI
#

import math
from Constants import *


# square
def sq( x ) :
    return x*x


# square root
def sqrt( x ) :
    return math.sqrt( x )


# min
def min( a, b ) :
    return a if a < b else b


# max 
def max( a, b ) :
    return a if a > b else b


# Asin safe
def asin_safe( x ) :
    return asin( max( -1, min( 1, x ) ) )


# sqrt_safe
def sqrt_safe( x ) :
    return math.sqrt( max( x, 0) )


# Atan2 safe
def atan2_safe( y, x ) :
    if y == 0 and x == 0 :
        return 0
    return math.atan2( y, x )


# Discriminant
def discr( a, b, c ) :
    return b**2 - 4*a*c


# Quadratic equation
def root( a, b, c, eps ) :
    if a == 0 :
        return NaR # this case should never happen
    d = discr( a, b, c )
    if ( d < 0 ) :
        return NaR # this case should never happen
    return ( -b + eps * math.sqrt( d ) / (2*a) )


# To range ( -pi, pi ]
def topi( x ) :
    while x <= -PI or PI < x :
        if ( x <= -PI ) :
            x += 2*PI
        else :
            x -= 2*PI
    return x


# To range [0, 360)
def to360( x ) :
    while x < 0 or 360 <= x :
        if x < 0 :
            x += 360
        else :
            x -= 360
    return x


# Radians to degrees in [0, 360)
def rad2deg( x ) :
    return to360( x * 180 / PI )


# Degrees to radians in (-pi, pi]
def deg2rad( x ) :
    return topi( x * PI / 180.0 )


# Degrees:Minutes to radians
def degmin2rad( d, m ) :
    return ( d + np.sign(d) * m / 60.0 ) * PI / 180.0


# Meters to nautical miles
def m2nm( x ) :
    return x / 1852


# Nautical miles to meters 
def nm2m( x ) :
    return x * 1852


# Knots to meters/seconds
def knots2msec( x ) :
    return nm2m(x) / 3600.0


# Meters/seconds to knots
def msec2knots( x ) :
    return m2nm(x) * 3600


# Feet to meters
def ft2m( x ) :
    return 1.609 * x / 5.28


# Meters to feet 
def m2ft( x ) :
    return 5.28 * x / 1.609


# Feets/minutes to meters/seconds
def ftmin2msec( x ) :
    return ft2m(x) / 60.0


# Meters/seconds to feets/minutes
def msec2ftmin( x ) :
    return m2ft(x) * 60 


# Sign ( 0 is positive )
def sign( x ) :
    return 1 if x >= 0 else -1


# ground speed to vx (trk [rad] in true North-clockwise convention) 
def gs2vx( gs, trk ) :
    return gs * sin(trk)


# ground speed to vy (trk [rad] in true North-clockwise convention)
def gs2vy( gs, trk ) :
    return gs * cos(trk)