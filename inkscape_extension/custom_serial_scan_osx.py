#!/usr/bin/env python

# Scan for custom serial implementations attached to USB
#
# Based on the eggbot scan code, made more generic for our purposes
#

import os
import re


DEV_TREE = '/dev'
USB_DEV_PREFIX = 'cu.usbmodem'

def findCustomSerial(devname, usb_vid, usb_pid):
	usbdata = os.popen( '/usr/sbin/system_profiler SPUSBDataType' ).read()
	tokens = re.split(devname, usbdata)
	for t in tokens[1:]:
		match = re.match( '.*?Location ID: 0x([0-9a-fA-F]+).*', t, re.M | re.S )
		if match != None:
			locid = int( match.group( 1 ), 16 )
			yield os.path.join( DEV_TREE,
					    '%s%x' % ( USB_DEV_PREFIX, ( ( locid >> 16 ) + 1 ) ) )

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
    
