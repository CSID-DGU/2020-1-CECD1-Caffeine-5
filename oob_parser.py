import struct
import serial
import time
import numpy as np
import math

class uartParserSDK():
    def __init__(self, type='(Legacy) 2D People Counting'):
        self.headerLength = 52
        self.magicWord = 0x708050603040102
        self.maxPoints = 1150
        #data storage
        self.pcPolar = np.zeros((5,self.maxPoints))
        self.pcBufPing = np.zeros((5,self.maxPoints))
        self.numDetectedObj = 0
        self.targetBufPing = np.ones((10,20))*-1
        self.indexBufPing = np.zeros((1,self.maxPoints))
        self.classifierOutput = []
        self.frameNum = 0
        self.missedFrames = 0
        self.byteData = bytes(1)
        self.oldData = []
        self.indexes = []
        self.numDetectedTarget = 0
        self.fail = 0
        self.unique = []
        self.getUnique = 0
        self.saveBinary = 0

    def polar2Cart(self):
        self.pcBufPing = np.empty((5,self.numDetectedObj))
        for n in range(0, self.numDetectedObj):
            self.pcBufPing[1,n] = self.pcPolar[0,n]*math.cos(self.pcPolar[1,n])  #y
            self.pcBufPing[0,n] = self.pcPolar[0,n]*math.sin(self.pcPolar[1,n])  #x
        self.pcBufPing[2,:self.numDetectedObj] = 0                               #Z is zero in 2D case
        self.pcBufPing[3,:] = self.pcPolar[2,0:self.numDetectedObj] #doppler
        self.pcBufPing[4,:] = self.pcPolar[3,0:self.numDetectedObj] #snr

    def tlvHeaderDecode(self, data):
        tlvType, tlvLength = struct.unpack('2I', data)
        return tlvType, tlvLength

    def parseDetectedObjects(self, data, tlvLength):
        objSize = struct.calcsize('4f')
        self.numDetectedObj = int((tlvLength)/16)
        for i in range(self.numDetectedObj):
            try:
                self.pcPolar[0,i], self.pcPolar[1,i], self.pcPolar[2,i], self.pcPolar[3,i] = struct.unpack('4f',data[:objSize])
                data = data[16:]
            except:
                self.numDectedObj = i
                break
        self.polar2Cart()

    def parseDetectedTracks(self, data, tlvLength):
        targetSize = struct.calcsize('I6f9ff')
        self.numDetectedTarget = int(tlvLength / targetSize)
        targets = np.empty((13,self.numDetectedTarget))
        for i in range(self.numDetectedTarget):
            targetData = struct.unpack('I6f9ff',data[:targetSize])
            targets[0,i]    =   int(targetData[0])
            targets[1:3,i]  =       targetData[1:3]
            targets[3,i]    =       0
            targets[4:6,i]  =       targetData[3:5]
            targets[6,i]    =       0
            targets[7:9,i]  =       targetData[5:7]
            targets[9,i]    =       0
            targets[10:12,i]=       [0.75, 0.75]
            targets[12,i]   =       1
            print("--------Target List---------")
            print("# of Detected Target: \t", self.numDetectedTarget)
            print("Target ID: \t", int(targets[0,i]))
            print("좌표 [X, Y]: \t\t[", round(targets[1, i], 2), ',', round(targets[2, i], 2),']')
            print("상대속도 [X, Y]: \t[", round(targets[4, i], 2), ',',  round(targets[5, i], 2),']')
            print("절대속도: \t\t", round(math.sqrt(pow(targets[4, i], 2) + pow(targets[5, i], 2)), 2),'m/s')
            print("상대가속도 [X, Y]: \t[", round(targets[7, i], 2), ',',  round(targets[8, i], 2),']')
            # TODO: 방향 판단하기
            data = data[targetSize:]    
        self.targetBufPing = targets           

    def parseTargetAssociations(self, data):
        targetSize = struct.calcsize('B')
        numIndexes = int(len(data) / targetSize)
        self.indexes = []
        self.unique = []
        try:
            for i in range(numIndexes):
                ind = struct.unpack('B', data[:targetSize])
                self.indexes.append(ind[0])
                data = data[targetSize:]
            if (self.getUnique):
                uTemp = self.indexes[math.ceil(numIndexes / 2):]
                self.indexes = self.indexes[:math.ceil(numIndexes / 2)]
                for i in range(math.ceil(numIndexes / 8)):
                    for j in range(8):
                        self.unique.append(getBit(uTemp[i], j))
        except: print('TLV Index Parse Fail')

    def parseClassifierOutput(self, data):
        clOutSize = struct.calcsize('Ii')
        self.classifierOutput = np.zeros((2, self.numDetectedTarget))
        for i in range(self.numDetectedTarget):
            self.classifierOutput[0, i], self.classifierOutput[1, i] = struct.unpack('Ii', data[:clOutSize])
            data = data[clOutSize:]

    def tlvHeader(self, data):
        self.targetBufPing = np.zeros((12,1))
        self.pcBufPing = np.zeros((5,self.maxPoints))
        self.indexes = []
        frameNum = -1
        self.numDetectedTarget = 0
        self.numDetectedObj = 0
        while (1):
            try: magic, version, platform, timestamp, packetLength, frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum =  struct.unpack('Q10I2H', data[:self.headerLength])
            except:
                self.fail = 1
                return data
            if (magic != self.magicWord): data = data[1:]
            else: break
        if (self.frameNum != frameNum):
            self.missedFrames += 1
            self.frameNum = frameNum
        self.frameNum += 1
        if (len(data) < packetLength):
            ndata = self.dataCom.read(packetLength - len(data))
            if (self.saveBinary): self.oldData += ndata
            data += ndata

        data = data[self.headerLength:]

        for i in range(numTLVs):
            try: tlvType, tlvLength = self.tlvHeaderDecode(data[:8])
            except:
                print('read fail: not enough data')
                self.missedFrames += 1
                self.fail=1
                break
            try:
                data = data[8:]
                if (tlvType == 6):      self.parseDetectedObjects(data[:tlvLength], tlvLength - 8)
                elif (tlvType == 7):    self.parseDetectedTracks(data[:tlvLength], tlvLength - 8)
                elif (tlvType == 8):    self.parseTargetAssociations(data[:tlvLength - 8])
                data = data[tlvLength - 8:]
            except:
                print('Not enough data')
                print('Data length: ', len(data))
                print('Reported Packet Length: ', packetLength)
                self.fail=1
                return data
        return data

    def readAndParseUart(self):
        data = self.dataCom.read(4666)
        if (self.byteData is None): self.byteData = data
        else: self.byteData += data
        if (self.saveBinary): self.oldData += data
        self.byteData = self.tlvHeader(self.byteData)

        return self.pcBufPing, self.targetBufPing, self.indexes, self.numDetectedObj, self.numDetectedTarget, self.frameNum, self.fail, self.classifierOutput

    def connectComPorts(self, uartCom, dataCom):
        self.uartCom = serial.Serial(uartCom, 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.3)
        self.dataCom = serial.Serial(dataCom, 921600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.025)
        self.dataCom.reset_output_buffer()
        print('Connected')

    def sendCfg(self, cfg):
        for line in cfg:
            time.sleep(.1)
            self.uartCom.write(line.encode())
            ack = self.uartCom.readline()
            print(ack)
            ack = self.uartCom.readline()
            print(ack)
        time.sleep(3)
        self.uartCom.reset_input_buffer()
        self.uartCom.close()

    def sendLine(self, line):
        self.uartCom.write(line.encode())
        ack = self.uartCom.readline()
        print(ack)
        ack = self.uartCom.readline()
        print(ack)

        
def getBit(byte, bitNum):
    mask = 1 << bitNum
    if (byte&mask): return 1
    else: return 0