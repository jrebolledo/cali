# By Marco Sangalli <sangalli.marco@google.com>

from serial import Serial
import time
from threading import Thread

##########################################################
def a2bin(n):
    o=[0,0,0,0,0,0,0,0]
    c=0
    while (n>>c)>0:
        o[c]=n>>c & 1     
        c+=1
    o.reverse()
    return o
    
def a2hex(data):
    e=[ ord(x) for x in data]
    return e 

##########################################################
class UARTApi(Thread):
    def __init__(self,dev="/dev/ttyUSB0",baud=9600,verbose=False):
        Thread.__init__(self)
        #        
        self.ser=Serial(dev,baud)
        self.ser.flushInput()
        self.ser.flushOutput()
        #
        self.verbose    =   verbose
        #
        self.mac_list_detected  =   []
        self.conversiondata = 0
        # TYPE OF device
        self.tod = 0
        self.bufferIn = {}
        self.cnt = 0
        #
        self.start()
        
    #read data    
    def run(self):
        while 1:
            res=self.ser.read()
            #print res
            #header found
            if ord(res)==0x7e:
                pack=self.unpackMsg()
                if pack:
                    self.bufferIn = pack
                    self.onData(self.bufferIn)
                 
            time.sleep(0.0001)
                
    def APIescapeMake(self, data):
        datacheck = []
        for i in range(len(data)):
            if data[i] == 0x13 or data[i] == 0x11 or data[i] == 0x7E or data[i] == 0x7D:
                datacheck.append(0x7D)
                datacheck.append(0x20^data[i])
            else:
                datacheck.append(data[i])
        
        return datacheck
    
    def APIescapeMake1(self, data):
        datacheck = []
        if data == 0x13:
            datacheck.append(0x7D)
            datacheck.append(0x20^data)
        else:
            datacheck.append(data)
        
        return datacheck

    def APIescapeCHECK(self):
        temp1 = ord(self.ser.read())
        #print hex(temp1)
        if temp1 == 0x7d:
            temp1 = ord(self.ser.read()) ^ 0x20
          
        return temp1
    
    def unpackMsg(self):
        #msg len
        #hlen=self.APIescapeCHECK()
        LEN_PARAMS  =   self.APIescapeCHECK()
        METHOD      =   self.APIescapeCHECK()
        PARAMS      = []
        for param in range(LEN_PARAMS):
            PARAMS.append(self.APIescapeCHECK())
        
        ID = self.APIescapeCHECK()
        
        CHECKSUM = self.APIescapeCHECK()
        
        #test checksum
        if self.verifychecksum([METHOD]+PARAMS+[ID]+[CHECKSUM]): 
            #return None
            pass
        else:
            pass
            #print 'dato cagado'
        #unpack ################################
        
        return {'method':METHOD,'params':PARAMS,'id':ID}

    def onData(self,pkg):
        print "*"*10
        print "METHOD:%s"%pkg['method']
        print "PARAMS:%s"%pkg['params']
        print "ID:%s"%pkg['id']
        print "*"*10
        return None

        
    #calculate checksum    
    def checksum(self,pkg):
        s=0
        for i in pkg:
            s=s+i
        diff=s&0xff
        return [0xff-diff]
    
    def verifychecksum(self,pkg):
        s=0
        for i in pkg:
            s=s+i
        #
        if (0xff-s)==0:
            return True
        else:
            return False    
        
    #pack structure
    def packAndSend(self,pkg):
        #checksum
        #merge data + checksum
        ff = [pkg['method']]+pkg['params']+[pkg['id']]
        ff.extend(self.checksum([pkg['method']]+pkg['params']+[pkg['id']]))
        
        #attach pream
        out=[len(pkg['params'])]

        #attach data + checksum to out
        out.extend(ff)
        
        #print out
        out2 = [0x7e] + self.APIescapeMake(out)
        #print out2
        #print out2
        outstr=("").join([chr(x) for x in out2])
        #send string
        self.ser.write(outstr)



