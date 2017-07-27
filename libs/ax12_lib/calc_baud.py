#
#
#

def pic_speed(v):
    return (77385000.0/2)/(16*(v+1))

def ax12_speed(v):
    return 2000000.0/(v+1)

if __name__ == "__main__":

    pairs = []
    # generate all pairs
    for pv in range(0,256):
        for av in range(0,256):
            p_speed = pic_speed(pv)
            a_speed = ax12_speed(av)
            pairs.append( (pv, av, p_speed, a_speed, abs(p_speed - a_speed)) )

    # now compute min
    min_val = pairs[0][4]
    min_val_index = 0
    for i in range(1,len(pairs)):
        if pairs[i][4] < min_val:
            min_val = pairs[i][4]
            min_val_index = i

    print "Minimum Error At:"
    #print pairs[min_val_index]
    print "AX12 Constant", pairs[min_val_index][1]
    print "dsPIC Constant", pairs[min_val_index][0]
    print
    print
    print "Other possible values with error less than 1%"
    print "----------------+-----------------+-----------"
    print "       AX12     |       dsPIC     |"
    print "  Speed   Value |   Speed  Value  |  Error (%)"
    print "----------------+-----------------+-----------"
    for i in range(0,len(pairs)):
        (pv, av, p_speed, a_speed, diff) = pairs[i]
        if p_speed > a_speed:
            ratio = p_speed / a_speed
        else:
            ratio = a_speed / p_speed

        if (ratio < 1.01)and(ratio > 0.99):
            print " %8d %4d  | %8d %4d   |   %5.2f" % (int(a_speed), av, int(p_speed), pv, abs(1-ratio)*100)
            #print pairs[i], ratio



