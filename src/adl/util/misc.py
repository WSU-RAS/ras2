#1/usr/bin/env python

from uuid import getnode

def get_mac():
    mac = getnode()
    serial = ''.join(("%012X" % mac)[i:i+2] for i in range(0, 12, 2))
    return serial
