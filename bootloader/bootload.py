#!/usr/bin/python
import sys
import os
if os.name=='nt':       # Windows
   import _winreg as winreg
import getopt
import subprocess

def findSerialPorts():
    '''
    Returns the serial ports of the motes connected to the computer.
    
    :returns: A list of tuples (name,baudrate) where:
        - name is a strings representing a serial port, e.g. 'COM1'
        - baudrate is an int representing the baurate, e.g. 115200
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
                        serialports.append( (str(val[1])) )
    
    return serialports
    
def usage():
    print '=== example : \n'
    print '    bootload.py --image=<image>\n'
    print '=== Or just type download bootload.py using image: flashflood\project\draughts\Debug\Exe\draughts.hex \n'
    

# =========================== main ============================================
def main():
    
    # get options
    cmd,start,end = None, None, None
    image = None
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'i:', ['image='])
    except getopt.GetoptError:
        usage()
        sys.exit(2)
        
    for opt, arg in opts:
        if opt in ('-h', '--help'):
            usage()
            sys.exit(2)
        elif opt in ('-i', '--image'):
            image = arg
        else:
            sys.exit(2)
        
    # check valid fo options
    if image == None:
        image = "..\project\ft_autoack\Debug\Exe\ft_autoack.a43" 
    
    for port in findSerialPorts():
        subprocess.Popen("python bsl --telosb -c {0} -r -e -I -p {1}".format(port,image))


# ==== start from here
if __name__ == '__main__':
    main()