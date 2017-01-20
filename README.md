# dynamic_marker
This package implements a dynamic fiducial marker system for automatic quadcopter landing.
This system control the display of a marker on a external screen and implements the decision process in order to select the type and size of the marker to be displayed.


# System description and required packages

TUD_COOP_UV
is used as the basis package for quadcopter marker tracking

Modified ar_sys package
for the detection of Aruco Markers with the implementation of services for dynamic reconfiguration of the markers to be detected.

Modified whycon package
for the detection of whycon markers with the implementation of services for dynamic reconfiguration of the markers to be detected.

Modified cob_fiducials
for the detection of pitag markers  with the implementation of services for dynamic reconfiguration of the markers to be detected.

TODO: Create a node in charge of marker recognition for all kinds of markers. (maybe not worth the time).
