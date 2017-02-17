#!/usr/bin/python
import os
import _winreg as winreg
import threading
import serial

SERIALPORTMAP = {
    'COM20': 'sensing node',
    'COM16': 'hop 1, A',
    'COM19': 'hop 1, B',
    'COM5':  'hop 2, A',
    'COM21': 'hop 2, B',
    'COM18': 'sink node',
}

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

class ProtectedPrinter(object):
    def __init__(self):
        self.screenLock = threading.Lock()
    def pprint(self,s):
        with self.screenLock:
            print s

class SerialListerer(threading.Thread):
    def __init__(self,serialport,pp):
        
        # store params
        self.serialport = serialport
        self.pp = pp
        
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
                self.pp.pprint('{0:>15}: {1}'.format(SERIALPORTMAP[self.serialport],''.join(self.rxBuf)))
                self.rxBuf  = []
            else:
                self.rxBuf += [c]

# =========================== main ============================================
def main():
    
    pp = ProtectedPrinter()
    serialports = findSerialPorts()
    print serialports
    for serialport in serialports:
        SerialListerer(serialport,pp)
    
    while True:
        command = raw_input('Press \'q\' to quit\n')
        if command=='q':
            return

# ==== start from here
if __name__ == '__main__':
    main()
