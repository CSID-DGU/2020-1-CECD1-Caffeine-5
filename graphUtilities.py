#functions for creating ellipsoids to represent tracks
import numpy as np

#helper funtions
def getBoxVertices(xl,yl,zl,xr,yr,zr):
    verts = np.zeros((8,3))
    verts[0,:] = [xl,yl,zl]
    verts[1,:] = [xr,yl,zl]
    verts[2,:] = [xl,yr,zl]
    verts[3,:] = [xr,yr,zl]
    verts[4,:] = [xl,yl,zr]
    verts[5,:] = [xr,yl,zr]
    verts[6,:] = [xl,yr,zr]
    verts[7,:] = [xr,yr,zr]
    return verts

def getBoxLinesFromVerts(verts):
    lines = np.zeros((24,3))
    #v0
    lines[0]= verts[0]
    lines[1]= verts[1]
    lines[2]= verts[0]
    lines[3]= verts[2]
    lines[4]= verts[0]
    lines[5]= verts[4]
    #v3
    lines[6]= verts[3]
    lines[7]= verts[1]
    lines[8]= verts[3]
    lines[9]= verts[2]
    lines[10]=verts[3]
    lines[11]=verts[7]
    #v5
    lines[12]=verts[5]
    lines[13]=verts[4]
    lines[14]=verts[5]
    lines[15]=verts[7]
    lines[16]=verts[5]
    lines[17]=verts[1]
    #v6
    lines[18]=verts[6]
    lines[19]=verts[2]
    lines[20]=verts[6]
    lines[21]=verts[4]
    lines[22]=verts[6]
    lines[23]=verts[7]
    return lines

def getBoxLines(xl,yl,zl,xr,yr,zr):
    verts = getBoxVertices(xl,yl,zl,xr,yr,zr)
    return getBoxLinesFromVerts(verts)

def getBoxLinesCoords(x,y,z,xrad=0.25,yrad=0.25,zrad=0.5):
    xl=x-xrad
    xr=x+xrad
    yl=y-yrad
    yr=y+yrad
    zl=z-zrad
    zr=z+zrad
    verts = getBoxVertices(xl,yl,zl,xr,yr,zr)
    return getBoxLinesFromVerts(verts)

def getSquareLines(xl,yL,xr,yr,z):
    verts = np.zeros((5,3))
    verts[0,:] = [xl,yL,z]
    verts[1,:] = [xr,yL,z]
    verts[2,:] = [xr,yr,z]
    verts[3,:] = [xl,yr,z]
    verts[4,:] = [xl,yL,z]
    return verts