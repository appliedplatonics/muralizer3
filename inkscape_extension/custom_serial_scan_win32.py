#!/usr/bin/env python

# Scan for custom serial implementations attached to USB
#
# Based on the eggbot scan code, made more generic for our purposes
#

import _winreg
import re

def findCustomSerial(devname, usb_vid, usb_pid):
    hReg = _winreg.ConnectRegistry( None, _winreg.HKEY_LOCAL_MACHINE )

    suffixes = [ "" ] + [ "&MI_%02x" % i for i in range(20) ]

    for i,s in enumerate(suffixes):
        try:
            curkey = "SYSTEM\CurrentControlSet\Enum\USB\VID_%s&PID_%s%s" % (usb_vid, usb_pid, s)
            hKey = _winreg.OpenKey( hReg, curkey)
        except WindowsError:
            if i == len(suffixes) - 1:
                _winreg.CloseKey( hReg )        
                raise StopIteration
            else:
                continue

        nKeys,nVals,nTime = _winreg.QueryInfoKey( hKey )
        for i in range( nKeys ):
            dev = _winreg.EnumKey( hKey, i )
            hKey2 = _winreg.OpenKey( hKey, dev )
            try:
                fname,t = _winreg.QueryValueEx( hKey2, "FriendlyName" )
                match = re.search( r".*\((.*)\)$", fname )
                yield match.group( 1 )
            except GeneratorExit:
                _winreg.CloseKey( hKey )
                _winreg.CloseKey( hReg )
                raise StopIteration
            except:
                pass
            finally:
                _winreg.CloseKey( hKey2 )

    # The next two lines may not executed when our caller
    # succeeds in finding an Eggbot device: in that case
    # the caller may do a "break" which then triggers the
    # "except GeneratorExit" clause above.
    _winreg.CloseKey( hKey )
    _winreg.CloseKey( hReg )

def findSerialPorts():
    found = 0
    hReg = _winreg.ConnectRegistry( None, _winreg.HKEY_LOCAL_MACHINE )
    hKey = _winreg.OpenKey( hReg, r"SOFTWARE\Microsoft\Windows NT\CurrentVersion\Ports" )
    nKeys,nVals,nTime = _winreg.QueryInfoKey( hKey )
    for i in range( nVals ):
        n,v,t = _winreg.EnumValue( hKey, i )
        if n[0:3] == 'COM':
            found = 1
            try:
                if n[-1] == ':':
                    yield n[:-1]
                else:
                    yield n
            except GeneratorExit:
                _winreg.CloseKey( hKey )
                _winreg.CloseKey( hReg )
                raise StopIteration

    # The next two lines may not executed when our caller
    # succeeds in finding an Eggbot device: in that case
    # the caller may do a "break" which then triggers the
    # "except GeneratorExit" clause above.
    _winreg.CloseKey( hKey )
    _winreg.CloseKey( hReg )

    # If we didn't find anything, then produce COM1, COM2, COM3, ..., COM99
    if found == 0:
        for i in range( 1, 100 ):
            yield "COM" + str( i )

if __name__ == '__main__':
    for board,vid,pid in [
            ("EiBotBoard"      , "04D8", "FD92"),
            ("Muralizer"       , "F054", "0100"),
            ("Leonardo"        , "2341", "8036"),
            ("LUFA CDC-ACM"    , "03EB", "204E"),            
             ]:
        print "Looking for %ss" % (board)
        for port in findCustomSerial(board, vid, pid):
            print "  ", port

    print "Looking for COM ports"
    for port in findSerialPorts():
        print "  ", port
    

    import time
    time.sleep(30)
