# -*- coding: mbcs -*-
# Do not delete the following import lines
from abaqus import *
from abaqusConstants import *
import __main__
### List of parameters for developing the deck model
## Basic parameters

# deck plate
t_dp = 5/8.0    ## thickness of deck plate
tp = 6.0        ## transverse projection of deck plate beyond the last rib
lp = 12.0       ## longitudinal projection of deck plate beyond last FB

# ribs
sr = 12.0       ## spacing of ribs
nro = 1         ## number of ribs in the outer overhang bay
nri = 8         ## number of ribs in between two girders
tr = 1/2.0      ## thickness of ribs
hr = 12.0       ## height of ribs

# girders
ng = 2          ## number of girders
dg = 48.0       ## depth of girder
bf = 24.0       ## breadth of flange
tf = 2.0        ## thickness of girder flange
tw = 3/4.0      ## thickness of girder web

# floor beams
dfb = 30.0      ## depth of FB
s_fb = 72.0     ## spacing of FB
tw_fb = 1/2.0   ## thickness of FB web
tf_fb = 1.0     ## thickness of FB flange
nfb = 3         ## number of FBs
#amol's temporary variables
flangeWidth = 10

# Circular cutout
rc = 1.5        ## radius of circular cutout
cc = 0.0        ## center of circular cutout from rib soffit - vertical distance

# apple cutout
rcl = 1.75      ## radius of lower arc
rcu = 9/16.0    ## radius of upper arc
rcv = 1.0       ## vertical distance between arcs
rch = 3/8.0     ## horizontal lip of cutout at weld termination
co = 0.25        ## center of apple cutout from rib soffit - vertical distance



## Derived parameters
sg = (nri + 1)*sr   ## spacing of girders
do = nro*sr + tp    ## transverse deck overhang beyound girders
B = (ng - 1)*sg + 2*do      ## Width of deck/deck plate
L = (nfb -1)*s_fb + 2*lp     ## Lenght of deck/deck plate

def verticalPartitions():
    import section
    import regionToolset
    import displayGroupMdbToolset as dgm
    import part
    import material
    import assembly
    import step
    import interaction
    import load
    import mesh
    import optimization
    import job
    import sketch
    import visualization
    import xyPlot
    import displayGroupOdbToolset as dgo
    import connectorBehavior
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[47], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[47], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#2 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[115], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[115], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[68], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[68], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#2 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[118], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[118], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#4 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[110], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[110], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#8 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[96], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[96], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#10 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[76], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[76], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#20 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[56], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[56], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#40 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[36], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[36], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#200 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[289], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[289], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#400 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[307], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[307], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#100 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[266], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[266], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#200 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[294], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[294], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#200 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[298], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[298], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[10], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[10], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#400 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[319], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[319], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#800 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[10], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[10], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#800 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[335], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[335], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[10], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[10], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1000 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[351], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[351], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#2000 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[10], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[10], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#2000 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[367], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[367], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[10], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[10], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#4000 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[383], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[383], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[10], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[10], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#2000000 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[546], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[542], rule=CENTER))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#800000 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[519], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[525], rule=CENTER))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1000 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[353], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[351], rule=CENTER))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#800000 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[336], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[334], rule=CENTER))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#400 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[319], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[314], rule=CENTER))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#800000 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[302], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[297], rule=CENTER))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#100 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[285], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[280], rule=CENTER))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#80 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[268], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[263], rule=CENTER))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#4000 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[121], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[118], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#20 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[427], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[430], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#40000000 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[172], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[169], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#100 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[458], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[461], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#40000 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[224], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[221], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#800 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[489], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[492], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#100000 ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[275], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[272], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#4000 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[520], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[523], rule=MIDDLE))


def horizontalPartitions():
    import section
    import regionToolset
    import displayGroupMdbToolset as dgm
    import part
    import material
    import assembly
    import step
    import interaction
    import load
    import mesh
    import optimization
    import job
    import sketch
    import visualization
    import xyPlot
    import displayGroupOdbToolset as dgo
    import connectorBehavior
    p = mdb.models['Model-1'].parts['FBInside']
    e = p.edges
    pickedEdges = e.getSequenceFromMask(mask=('[#0:16 #1000 ]', ), )
    p.PartitionEdgeByParam(edges=pickedEdges, parameter=0.269422467549642)
    p = mdb.models['Model-1'].parts['FBInside']
    e = p.edges
    pickedEdges = e.getSequenceFromMask(mask=('[#0:16 #2000 ]', ), )
    p.PartitionEdgeByParam(edges=pickedEdges, parameter=0.34702829373028)
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#ffffada9 #3fbff ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(point=v[348], normal=e[524], 
        cells=pickedCells)
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1c3878f7 #3a0340e8 #df7b380 ]', ), 
        )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(point=v1[137], normal=e1[190], 
        cells=pickedCells)
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#3cf3ffff #1cf #580000 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[624], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[298], rule=CENTER))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#0:2 #fffff000 #17fffff ]', ), )
    e1, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[817], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[1512], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBInside']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=(
        '[#0:2 #87000000 #81c3 #49200000 #18104000 #1c008 ]', ), )
    e, v, d = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(point=v[433], normal=e[766], 
        cells=pickedCells)

def meshFBInside():
    import section
    import regionToolset
    import displayGroupMdbToolset as dgm
    import part
    import material
    import assembly
    import step
    import interaction
    import load
    import mesh
    import optimization
    import job
    import sketch
    import visualization
    import xyPlot
    import displayGroupOdbToolset as dgo
    import connectorBehavior
    p = mdb.models['Model-1'].parts['FBInside']
    p.seedPart(size=1.5, deviationFactor=0.1, minSizeFactor=0.1)
    p.generateMesh()

verticalPartitions();
horizontalPartitions();
meshFBInside();
