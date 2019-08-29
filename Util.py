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


const double TauMin = 0.0001; // A minimum time for positive tau

const double NaR = 9999999;   // An arbitrary large number

const double Pi = 3.141592654; // Value of Pi 

# square
def sq( x ) :
    return x*x

# # min
# def min( a, b ) :
#     return a if a < b else b


# # max 
# def max( a, b ) :
#     return a if a > b else b


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
        return None 