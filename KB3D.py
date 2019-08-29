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

