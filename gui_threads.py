from PyQt5.QtCore import QThread, pyqtSignal
from graphUtilities import *
import pyqtgraph as pg
import numpy as np

class parseUartThread(QThread):
        fin = pyqtSignal('PyQt_PyObject')

        def __init__(self, uParser):
                QThread.__init__(self)
                self.parser = uParser

        def run(self):
                pointCloud = self.parser.readAndParseUart()
                self.fin.emit(pointCloud)


class sendCommandThread(QThread):
        done = pyqtSignal()
        def __init__(self, uParser, command):
                QThread.__init__(self)
                self.parser = uParser
                self.command = command

        def run(self):
            self.parser.sendLine(self.command)
            self.done.emit()


class updateQTTargetThread3D(QThread):
    done = pyqtSignal()

    def __init__(self, pointCloud, targets, indexes, scatter, pcplot, numTargets, ellipsoids, coords, classifierOut=[], zRange=[-3, 3], gw=[], colorByIndex=False, drawTracks=True, bbox=[0,0,0,0,0,0], bbox_en=0):
        QThread.__init__(self)
        self.pointCloud = pointCloud
        self.targets = targets
        self.indexes = indexes
        self.scatter = scatter
        self.pcplot = pcplot
        self.colorArray = ('r','g','b','w')
        self.numTargets = numTargets
        self.ellipsoids = ellipsoids
        self.coordStr = coords
        self.classifierOut = classifierOut
        self.zRange = zRange
        self.gw = gw
        self.colorByIndex = colorByIndex
        self.drawTracks = drawTracks
        self.bbox = bbox
        self.bbox_en = bbox_en

    def drawTrack(self, index):
        #get necessary target data
        tid = int(self.targets[0,index])
        x = self.targets[1,index]
        y = self.targets[2,index]
        z = self.targets[3,index]

        edge_color = pg.glColor(self.colorArray[tid%3])
        track = self.ellipsoids[tid]
        if (len(self.classifierOut) != 0):
            try:
                dTID = self.classifierOut[0].tolist()
                posit = dTID.index(tid)
                decision = self.classifierOut[1,posit]
            except Exception as ex:
                print ('Cannot find tid ', tid, ' in list:')
                print (dTID)
                print(ex)
            if(decision != 1):
                edge_color = pg.glColor('w')
        mesh = getBoxLinesCoords(x,y,z)
        track.setData(pos=mesh,color=edge_color,width=3,antialias=True,mode='lines')
        track.setVisible(True)
        #add text coordinates
        ctext = self.coordStr[tid]
        ctext.setPosition(x,y,z)
        ctext.setVisible(True)

    def run(self):
        #sanity check indexes = points
        if (len(self.indexes) != np.shape(self.pointCloud)[1]) and (len(self.indexes)):
            print ('I: ',len(self.indexes), ' P: ',  np.shape(self.pointCloud)[1])
        for e in self.ellipsoids:
            if (e.visible()): e.hide()
        for c in self.coordStr:
            if (c.visible()): c.hide()

        toPlot = self.pointCloud[0:3,:].transpose()
        size = np.log2(self.pointCloud[4,:].transpose())
        colors = np.zeros((np.shape(self.pointCloud)[1], 4))
        if (self.colorByIndex):
            if (len(self.indexes) > 0):
                try:
                    for i in range(len(self.indexes)):
                        if (int(self.indexes[i]) < 100): color = pg.glColor(self.colorArray[int(self.indexes[i]) % 3])
                        else: color = pg.glColor(self.colorArray[3])
                        colors[i,:] = color[:]
                    self.scatter.setData(pos=toPlot, color=colors, size=size)
                except: self.scatter.setData(pos=toPlot, size=size)
            else: self.scatter.setData(pos=toPlot, size=size)
        else:
            for i in range(np.shape(self.pointCloud)[1]):
                zs = self.pointCloud[2,i]
                if (zs < self.zRange[0]) or (zs > self.zRange[1]):
                    colors[i]=pg.glColor('k')
                else:
                    colorRange = self.zRange[1]+abs(self.zRange[0])
                    zs = self.zRange[1] - zs
                    colors[i]=pg.glColor(self.gw.getColor(abs(zs/colorRange)))
            self.scatter.setData(pos=toPlot, color=colors, size=size)
        #graph the targets
        if (self.drawTracks):
            for i in range(self.numTargets):
                try:
                    self.drawTrack(i)
                except Exception as e:
                    print(e)
                    print('No Plot Update')
        self.done.emit()