#!/usr/bin/env python

# Scan for custom serial implementations attached to USB
#
# Based on the eggbot scan code, made more generic for our purposes
#

import os
import re

DEV_TREE = '/dev'
USB_DEV_PREFIX = 'ttyACM'
USB_DEVICE_TREE = '/sys/bus/usb/devices'


def findCustomSerial(devname, usb_vid, usb_pid):
    """Find only those USB devices that declare themselves to be EiBotBoards"""

    # find all USB devices whose product name is 'EiBotBoard'
    with os.popen( 'fgrep -l "%s" %s/*/product' % (board, USB_DEVICE_TREE) ) as pipe:
        for path in [ os.path.split( path )[0] for path in pipe.readlines()]:
            device = os.path.split( path )[1]

            # for each device endpoint ...
            for dir in os.listdir( path ):
                if dir.startswith( device ):

                    # find the endpoint that supports tty access
                    ttydir = os.path.join( USB_DEVICE_TREE, device, dir, 'tty' )
                    if os.path.exists( ttydir ):

                        # And emit each (the) interface name
                        for ttyname in os.listdir( ttydir ):
                            yield os.path.join( DEV_TREE, ttyname )
                            
def findSerialPorts():
    device_list = os.listdir( DEV_TREE )
    for device in device_list:
        if not device.startswith( USB_DEV_PREFIX ):
            continue
        yield os.path.join( DEV_TREE, device )

if __name__ == '__main__':
    for board,vid,pid in [
            ("EiBotBoard"      , "04D8", "FD92"),
            ("Muralizer"       , "F054", "0100"),
            ("Leonardo"        , "2341", "8036"),
            ("DX-SR8 interf"    , "03EB", "204E"),
             ]:
        print "Looking for %ss" % (board)
        for port in findCustomSerial(board, 0, 0):
            print "  ", port

    print "Looking for COM ports"
    for port in findSerialPorts():
        print "  ", port
    
