import math


def normalize_angle(p):
    if p > 180:
        return p - 360
    if p < - 180:
        return p + 360
    else:
        return p

theta = 0
target = int(raw_input("target>"))
error = target - theta
a_error = abs(error)

if a_error >= 90:
    error = normalize_angle(180 - error)
    a_error = abs(error)
    v_sign = -1.0
else:
    v_sign = 1.0

	
if error < 0:
    w_sign = -1.0
else:
    w_sign = 1.0

print error
if -v_sign*w_sign > 0:
    print "CW",
else:
    print "CCW",

if v_sign < 0:
    print "REV"
else:
    print "FWD"

