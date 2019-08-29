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

