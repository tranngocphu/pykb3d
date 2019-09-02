#
# LoS.cpp
# Release: KB3D++-3.a (03/28/08)
#
# Contact: Cesar Munoz (munoz@nianet.org)
# National Institute of Aerospace
# http://research.nianet.org/fm-at-nia/KB3D
#
# Unit conventions are explained in KB3D.cpp
#
# This file provides the function
#   vertical_recovery
# 
# The below Python implementation was done by Phu Tran @ ATMRI
#

from math import fabs
from Util import *

# Breaks symmetry when vertical speed is zero
def break_vz_symm( sx, sy, sz) :
    if sz > 0 :
        return 1
    return break_vz_symm( sx, sy, sz )


def sign_vz( sx, sy, sz, vz) :
    if sz*vz >= 0 and vz != 0 :
        return sign(vz)
    return break_vz_symm(sx,sy,sz)


def vertical_recovery( sx, sy, sz, voz, viz, H, t) :
    vz  = voz - viz
    nvz =  ( sign_vz( sx, sy, sz, vz ) * H - sz ) / t   
    if sz*vz >= 0 and fabs(vz) >= fabs(nvz) :
        return vz + viz    
    return nvz + viz