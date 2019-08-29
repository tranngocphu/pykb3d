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



