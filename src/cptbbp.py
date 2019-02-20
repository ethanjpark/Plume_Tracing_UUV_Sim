#!/usr/bin/env python

# Chemical Plume Tracing - Behaviour Based Planning Algorithm

"""
Initiate GOTO (starting position of searching)


FIND
If chemical plume not found:
	keep searching
elif chemical plume detected:
	go to TRACKIN


TRACKIN:
if chemical plume is being detected:
	stay in TRACKIN

elif chemical plume is lost:
	go to TRACKOUT


TRACKOUT:
check if source can be declared
If source declared:
	go to POST DECLARE MANEUVERS

elif source is not declared but chemicals detected:
	go to TRACKIN

elif source is not declared and chemicals not detected:
	go to REACQUIRE


REACQUIRE:
if chemical plume is detected:
	go to TRACKIN

elif chemical plume is not detected:
	go back to FIND
"""