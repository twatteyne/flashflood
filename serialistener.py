#!/usr/bin/python
import os
import _winreg as winreg
import threading
import serial

def findSerialPorts():
    '''
    Returns ['COM1','COM2']
    '''
    serialports = []
    
    if os.name=='nt':
        path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
        skip = False
        try :
            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
        except :
            # No mote is connected
            skip = True
        if not skip :
            for i in range(winreg.QueryInfoKey(key)[1]):
                try:
                    val = winreg.EnumValue(key,i)
                except:
                    pass
                else:
                    if   val[0].find('VCP')>-1:
                        serialports += [str(val[1])]
    
    return serialports

class SerialListerer(threading.Thread):
    def __init__(self,serialport):
        
        # store params
        self.serialport = serialport
        
        # local variables
        
        # start the thread
        threading.Thread.__init__(self)
        self.name     = 'SerialListerer_{0}'.format(self.serialport)
        self.daemon   = True
        self.start()
    
    def run(self):
        
        self.mote  = serial.Serial(self.serialport, 115200)
        self.rxBuf = []
        
        while True:
            c = self.mote.read(1)
            if c=='\n':
                print '{0}: {1}'.format(self.serialport,''.join(self.rxBuf))
                self.rxBuf  = []
            else:
                self.rxBuf += [c]

# =========================== main ============================================
def main():
    
    serialports = findSerialPorts()
    
    for serialport in serialports:
        SerialListerer(serialport)
    
    while True:
        command = raw_input('Press \'q\' to quit\n')
        if command=='q':
            return

# ==== start from here
if __name__ == '__main__':
    main()
