# -*- coding: mbcs -*-
# Do not delete the following import lines
from abaqus import *
from abaqusConstants import *
import __main__

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
bf_fb = 10.0    #flangeWidth
wr = 1/4.0
#floorbeam partition
boxLength=8

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
src = (sr - tr)		## clear spacing of ribs
srg = sr - (tr+tw)/2.0 		## clear spacing between girder and rib
tpc = tp - tr/2.0 	## clear transverse overhang


def polyline_sketch(m, pts):
    import sketch
    '''
    Draws a closed section by mulitple lines
    Takes the model and the list of points as arguments
    Returns the section
    '''
    s = m.ConstrainedSketch(name='__profile__', sheetSize=200.0)
    npts = len(pts)
    for i in range(npts): 
        s.Line(pts[i % npts], pts[(i+1) % npts])
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    return s

def deckPlate():
    '''
    Create deck plate
    '''
    import part
    import sketch
    ## import required deck parameters from ordparams.py
    #from ordparams import L, B, t_dp

    m = mdb.models['Model-1']

    pt1 = (-B/2.0, t_dp/2.0)
    pt2 = (B/2.0, -t_dp/2.0)
    s1 = m.ConstrainedSketch(name='__profile__', sheetSize=200.0)
    s1.rectangle(pt1, pt2)
    p = m.Part(name='DeckPlate', dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p.BaseSolidExtrude(sketch=s1, depth=L)
    s1.unsetPrimaryObject()
    session.viewports['Viewport: 1'].setValues(displayedObject=p)
    del m.sketches['__profile__']


def partitionDeck():
    '''
    Partition deck by datum planes;
    Create datum planes offset from the -x and z=0 faces
    '''
    import itertools
    #from ordparams import *

    p = mdb.models['Model-1'].parts['DeckPlate']
    c, f = p.cells, p.faces

    ## create datum planes in the transverse direction
    # -x face taken as reference
    ref_face = f[f.findAt(((-B/2.0, 0.0, L/2.0),))[0].index]
    offset = 0.0
    incr = [tpc, tr, srg, tw, srg]  # offset distances from reference face
    for inc in incr:
        offset += inc
        p.DatumPlaneByOffset(ref_face, SIDE2, offset)
    for nr in range(nri):
        offset += tr
        p.DatumPlaneByOffset(ref_face, SIDE2, offset)
        if nr < nri-1 :
            offset += src
            p.DatumPlaneByOffset(ref_face, SIDE2, offset)
    for inc in incr[-1:0:-1]:
        offset += inc
        p.DatumPlaneByOffset(ref_face, SIDE2, offset)

    ## create datum planes in the transverse direction
    # z=0 face taken as reference
    ref_face = f[f.findAt(((0.0, 0.0, 0.0),))[0].index]
    for j in range(nfb):
        offset1 = lp + j*s_fb
        for i in [-0.5, 0.5]:
            for loc in [bf_fb, tw_fb]:
                offset = offset1 + i*loc
                p.DatumPlaneByOffset(ref_face, SIDE2, offset)

    ## create partitions by datum planes
    d = p.datums 
    for k in d.keys():
        c = p.cells
        p.PartitionCellByDatumPlane(c, d[k])
def Girder():

    bf_2 = bf/2.0
    tw_2 = tw/2.0

    pt1 = (-bf_2,0.)
    pt2 = (-bf_2, tf)
    pt3 = (-tw_2, tf)
    pt4 = (-tw_2, dg)
    pt5 = (tw_2, dg)
    pt6 = (tw_2, tf)
    pt7 = (bf_2, tf)
    pt8 = (bf_2,0.)

    m = mdb.models['Model-1']       ## Good idea to assign model to a variable


    ''' 
    Alternative cleaner way of drawing the section
    Define the pts as list
    Then call the polyline_sketch function defined earlier
    to draw the section.
    The polyline_sketch function allows drawing any polyline section
    defined by joining lines among points.
    '''
    pts = [(-bf_2,0.0),
            (-bf_2, tf),
            (-tw_2, tf),
            (-tw_2, dg),
            (tw_2, dg),
            (tw_2, tf),
            (bf_2, tf),
            (bf_2,0.0)]

    s = polyline_sketch(m, pts)
 
    p = mdb.models['Model-1'].Part(name='Girder', dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p.BaseSolidExtrude(sketch=s, depth=L)
    s.unsetPrimaryObject()
    session.viewports['Viewport: 1'].setValues(displayedObject=p)
    del mdb.models['Model-1'].sketches['__profile__']

def GirderPartition():
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
    p = mdb.models['Model-1'].parts['Girder']
    c = p.cells
    ##############Need to convert using parameters
    ##Middle Horizontal
    c = p.cells
    pickedCells = c
    p.PartitionCellByPlaneThreePoints(point1=(0,tf,0), point2=(L,tf,0),cells=pickedCells, point3=(0.0,tf,L))
    
    c = p.cells
    pickedCells = c
    p.PartitionCellByPlaneThreePoints(point1=(0,dg-dfb,0), point2=(L,dg-dfb,0),cells=pickedCells, point3=(0.0,dg-dfb,L))
    
    c = p.cells
    pickedCells = c
    p.PartitionCellByPlaneThreePoints(point1=(0,dg-dfb-tf_fb,0), point2=(L,dg-dfb-tf_fb,0),cells=pickedCells, point3=(0.0,dg-dfb-tf_fb,L))

    n=0
    while(n<nfb):
            c = p.cells
            pickedCells = c
            p.PartitionCellByPlaneThreePoints(point1=(0,0,lp+s_fb*n), point2=(L,0,lp+s_fb*n),cells=pickedCells, point3=(0,L,lp+s_fb*n))
            p.PartitionCellByPlaneThreePoints(point1=(0,0,lp+tw_fb+s_fb*n), point2=(L,0,lp+tw_fb+s_fb*n),cells=pickedCells, point3=(0,L,tw_fb+lp+s_fb*n))
            n = n +1
            
    p = mdb.models['Model-1'].parts['Girder']
    p.seedPart(size=1.0, deviationFactor=0.1, minSizeFactor=0.1)
    elemType1 = mesh.ElemType(elemCode=C3D20R, elemLibrary=STANDARD)
    elemType2 = mesh.ElemType(elemCode=C3D15, elemLibrary=STANDARD)
    elemType3 = mesh.ElemType(elemCode=C3D10, elemLibrary=STANDARD)
    p = mdb.models['Model-1'].parts['Girder']
    c = p.cells
    
    pickedCells = c
    pickedRegions =(pickedCells, )
    #mesh
    #p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, 
    #    elemType3))
    #p.generateMesh()

def create_innerFB():
    import part
    import sketch
    #from ordparams import *

    m = mdb.models['Model-1']
    p_name = 'FB_Inner'
    s1 = m.ConstrainedSketch(name='__profile__', sheetSize=200.0)
    s1.setPrimaryObject(option=STANDALONE)
    s1.Line(point1=(-tw_fb/2, dfb), point2=(tw_fb/2, dfb))
    s1.Line(point1=(tw_fb/2, dfb), point2=(tw_fb/2, 0.0))
    s1.Line(point1=(tw_fb/2, 0.0), point2=(bf_fb/2, 0.0))
    s1.Line(point1=(bf_fb/2, 0.0), point2=(bf_fb/2, -tf_fb))
    s1.Line(point1=(bf_fb/2, -tf_fb), point2=(-bf_fb/2, -tf_fb))
    s1.Line(point1=(-bf_fb/2, -tf_fb), point2=(-bf_fb/2, 0.0))
    s1.Line(point1=(-bf_fb/2, 0.0), point2=(-tw_fb/2, 0.0))
    s1.Line(point1=(-tw_fb/2, 0.0), point2=(-tw_fb/2, dfb))
    p = m.Part(name=p_name, dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p = m.parts[p_name]
    p.BaseSolidExtrude(sketch=s1, depth=sg)
    s1.unsetPrimaryObject()
    del m.sketches['__profile__']


def AppleCutSketch():
    import part
    import sketch
    #from ordparams import *

    m = mdb.models['Model-1']
    
    x1 = -tr/2.0; y1 = 0.0
    x2 = x1; y2 = -(hr-co-rcv)
    x3 = x2-rch; y3 = y2
    x4 = x3-2*rcu; y4 = y3
    x5 = x4; y5 = y4-rcv 
    x6 = -x5; y6 = y5
    x7 = -x4; y7 = y4
    x8 = -x3; y8 = y3
    x9 = -x2; y9 = y2 
    x10 = -x1; y10 = y1 


    s = m.ConstrainedSketch(name='__profile__', 
        sheetSize=200.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=STANDALONE)
    s.Line(point1=(x1, y1), point2=(x2, y2))
    s.Line(point1=(x2, y2), point2=(x3, y3))
    
    s.ArcByCenterEnds(center=((x3+x4)/2, y3), point1=(x3, y3), point2=(x4, y4), 
        direction=COUNTERCLOCKWISE)
    s.Line(point1=(x4, y4), point2=(x5, y5))
    s.ArcByCenterEnds(center=((x5+x6)/2, y5), point1=(x5, y5), point2=(x6, y6), 
        direction=COUNTERCLOCKWISE)
    s.Line(point1=(x6, y6), point2=(x7, y7))
    s.ArcByCenterEnds(center=((x7+x8)/2, y7), point1=(x7, y7), point2=(x8, y8), 
        direction=COUNTERCLOCKWISE)
    s.Line(point1=(x8, y8), point2=(x9, y9))
    s.Line(point1=(x9, y9), point2=(x10, y10))
    s.Line(point1=(x10, y10), point2=(x1, y1))

    m.sketches.changeKey(fromName='__profile__', 
        toName='AppleCutSketch')
    s.unsetPrimaryObject()


def create_AppleCutout_FBI():
    '''
    Create apple cutouts for half of inner FB ribs 
    '''
    import part
    import sketch
    #from ordparams import *

    m = mdb.models['Model-1']
    p = m.parts['FB_Inner']
    f1, e1 = p.faces, p.edges

    AppleCutSketch()

    '''
    implement findAt
    '''
    frontalPlane = f1.findAt(((tw_fb/2.0, dfb/2.0, (sg-tw)/2.0),))
    rightmostEdge = e1.findAt(((tw_fb/2.0, dfb/2.0, 0.0),))
    t = p.MakeSketchTransform(sketchPlane=frontalPlane[0], sketchUpEdge=rightmostEdge[0], 
        sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(tw_fb/2, dfb/2, 
        s_fb))
    s = m.ConstrainedSketch(name='__profile__', 
        sheetSize=294.18, gridSpacing=7.35, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=SUPERIMPOSE)
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)

    '''
    Revise for object list
    '''
    g = s.geometry
    old_geom_id = g.keys()
    s.retrieveSketch(sketch=m.sketches['AppleCutSketch'])
    g = s.geometry
    new_geom_id = g.keys()
    add_geom_id = list(set(new_geom_id) - set(old_geom_id))
    objList = tuple([g[i] for i in add_geom_id])
    s.move(vector=((sg/2)+(sr/2), dfb/2), objectList=objList)

    '''
    parameterize spacing
    '''
    s.linearPattern(geomList=objList, vertexList=(), number1=nri/2, spacing1=sr, 
        angle1=180.0, number2=1, spacing2=1, angle2=90.0)
    f, e = p.faces, p.edges
    
    frontalPlane = f.findAt(((tw_fb/2.0, dfb/2.0, (sg-tw)/2.0),))
    rightmostEdge = e.findAt(((tw_fb/2.0, dfb/2.0, 0.0),))
    p.CutExtrude(sketchPlane=frontalPlane[0], sketchUpEdge=rightmostEdge[0], sketchPlaneSide=SIDE1, 
        sketchOrientation=RIGHT, sketch=s, flipExtrudeDirection=OFF)
    s.unsetPrimaryObject()
    del m.sketches['__profile__']


def CircleCutSketch():
    import part
    import sketch
    #from ordparams import *

    m = mdb.models['Model-1']

    x1 = -tr/2 ; y1 = -(hr-rc)
    x2 = x1; y2 = cc
    x3 = tr/2 ; y3=y2
    x4 = x3; y4 = y1
    
    s1 = m.ConstrainedSketch(name='__profile__', 
        sheetSize=200.0)
    g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
    s1.setPrimaryObject(option=STANDALONE)

    s1.Line(point1=(x1, y1), point2=(x2, y2))
    s1.Line(point1=(x2, y2), point2=(x3, y3))
    s1.Line(point1=(x3, y3), point2=(x4, y4))
    s1.ArcByCenterEnds(center=(cc, -hr), point1=(-tr/2, -(hr-rc)), point2=(tr/2, 
        -(hr-rc)), direction=COUNTERCLOCKWISE)
    m.sketches.changeKey(fromName='__profile__', 
        toName='Circlecutsketch')
    s1.unsetPrimaryObject()

def create_CircleCutout_FBI():
    '''
    Create circular cutouts for half of inner FB ribs 
    '''
    import part
    import sketch
    #from ordparams import *

    m = mdb.models['Model-1']
    p = m.parts['FB_Inner']

    CircleCutSketch()

    f, e = p.faces, p.edges
    frontPlane = f.findAt(((tw_fb/2.0, dfb/2.0, (sg-tw)/2.0),))
    rightEdge = e.findAt(((tw_fb/2.0, dfb/2.0, 0.0),))
    t = p.MakeSketchTransform(sketchPlane=frontPlane[0], sketchUpEdge=rightEdge[0], 
        sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(-tw_fb/2, 
        dfb/2, s_fb))
    s = m.ConstrainedSketch(name='__profile__', 
        sheetSize=294.18, gridSpacing=7.35, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=SUPERIMPOSE)
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)

    g = s.geometry
    old_geom_id = g.keys()
    s.retrieveSketch(sketch=mdb.models['Model-1'].sketches['Circlecutsketch'])
    g = s.geometry
    new_geom_id = g.keys()
    add_geom_id = list(set(new_geom_id) - set(old_geom_id))
    objList = tuple([g[i] for i in add_geom_id])
    s.move(vector=(-(2*sr), dfb/2), objectList=objList)
    s.linearPattern(geomList=objList, vertexList=(), 
        number1=nri/2, spacing1=sr, angle1=0.0, number2=1, spacing2=1, 
        angle2=90.0)
    
    f1, e1 = p.faces, p.edges
    frontPlane = f1.findAt(((tw_fb/2.0, dfb/2.0, (sg-tw)/2.0),))
    rightEdge = e1.findAt(((tw_fb/2.0, dfb/2.0, 0.0),))
    
    p.CutExtrude(sketchPlane=frontPlane[0], sketchUpEdge=rightEdge[0], sketchPlaneSide=SIDE1, 
        sketchOrientation=RIGHT, sketch=s, flipExtrudeDirection=OFF)
    s.unsetPrimaryObject()
    del m.sketches['__profile__']


def create_outerFB(p_name = 'FB_Outer_A'):
    import part
    import sketch
    #from ordparams import *

    m = mdb.models['Model-1']
 
    s1 = m.ConstrainedSketch(name='__profile__', 
        sheetSize=200.0)
    s1.setPrimaryObject(option=STANDALONE)
    s1.Line(point1=(-tw_fb/2, dfb), point2=(tw_fb/2, dfb))
    s1.Line(point1=(tw_fb/2, dfb), point2=(tw_fb/2, 0.0))
    s1.Line(point1=(tw_fb/2, 0.0), point2=(bf_fb/2, 0.0))
    s1.Line(point1=(bf_fb/2, 0.0), point2=(bf_fb/2, -tf_fb))
    s1.Line(point1=(bf_fb/2, -tf_fb), point2=(-bf_fb/2, -tf_fb))
    s1.Line(point1=(-bf_fb/2, -tf_fb), point2=(-bf_fb/2, 0.0))
    s1.Line(point1=(-bf_fb/2, 0.0), point2=(-tw_fb/2, 0.0))
    s1.Line(point1=(-tw_fb/2, 0.0), point2=(-tw_fb/2, dfb))
    p = m.Part(name=p_name, dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p = m.parts[p_name]
    p.BaseSolidExtrude(sketch=s1, depth=(sr+(sr/2)))
    s1.unsetPrimaryObject()
    del m.sketches['__profile__']
    outerFBAppleCutout()

def create_outerFB_C():
    p_name = 'FB_Outer_C'
    create_outerFB(p_name)
    outerFBCircleCutout()


def outerFBAppleCutout():
    import part
    import sketch
    #from ordparams import *

    m = mdb.models['Model-1']
    p = m.parts['FB_Outer_A']

    f1, e1 = p.faces, p.edges
    frontalPlane = f1.findAt(((tw_fb/2.0, dfb/2.0, tw/2.0),))
    rightmostEdge = e1.findAt(((tw_fb/2.0, dfb/2.0, 0.0),))
    t = p.MakeSketchTransform(sketchPlane=frontalPlane[0], sketchUpEdge=rightmostEdge[0], 
        sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(tw_fb/2, dfb/2, 
        sr-(sr/4)))
    s = m.ConstrainedSketch(name='__profile__', 
        sheetSize=156.0, gridSpacing=3.9, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=SUPERIMPOSE)
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    old_geom_id = g.keys()
    s.retrieveSketch(sketch=m.sketches['AppleCutSketch'])
    new_geom_id = g.keys()
    add_geom_id = list(set(new_geom_id) - set(old_geom_id))
    objList = tuple([g[i] for i in add_geom_id])
    s.move(vector=(sr/4, dfb/2), objectList=objList)
    f, e = p.faces, p.edges
    p.CutExtrude(sketchPlane=frontalPlane[0], sketchUpEdge=rightmostEdge[0], sketchPlaneSide=SIDE1, 
        sketchOrientation=RIGHT, sketch=s, flipExtrudeDirection=OFF)
    s.unsetPrimaryObject()
    del m.sketches['__profile__']


def outerFBCircleCutout():
    import part
    import sketch
    #from ordparams import *

    m = mdb.models['Model-1']
    p = m.parts['FB_Outer_C']

    f1, e1 = p.faces, p.edges
    frontalPlane = f1.findAt(((tw_fb/2.0, dfb/2.0, tw/2.0),))
    rightmostEdge = e1.findAt(((tw_fb/2.0, dfb/2.0, 0.0),))
    t = p.MakeSketchTransform(sketchPlane=frontalPlane[0], sketchUpEdge=rightmostEdge[0], 
        sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(tw_fb/2, dfb/2, 
        sr-(sr/4)))
    s = m.ConstrainedSketch(name='__profile__', 
        sheetSize=156.0, gridSpacing=3.9, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=SUPERIMPOSE)
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    old_geom_id = g.keys()
    s.retrieveSketch(sketch=m.sketches['Circlecutsketch'])
    new_geom_id = g.keys()
    add_geom_id = list(set(new_geom_id) - set(old_geom_id))
    objList = tuple([g[i] for i in add_geom_id])
    s.move(vector=(-sr/4, dfb/2), objectList=objList)
    f, e = p.faces, p.edges
    p.CutExtrude(sketchPlane=frontalPlane[0], sketchUpEdge=rightmostEdge[0], sketchPlaneSide=SIDE1, 
        sketchOrientation=RIGHT, sketch=s, flipExtrudeDirection=OFF)
    s.unsetPrimaryObject()
    del m.sketches['__profile__']
#########################################################################################################################################

def FBverticalPartitions():
    import part
    import sketch
    #from ordparams import nri, sr, tw_fb, dfb

    m = mdb.models['Model-1']
    p = m.parts['FB_Inner']
    c = p.cells

    def part_midpt(rpts, interval):
        for j in range(1, rpts+1):
            e1 = p.edges
            z = (2*j-1)*interval/2.0
            pedge = e1[e1.findAt(((tw_fb/2.0, dfb, z),))[0].index]
            p.PartitionCellByPlanePointNormal(normal=pedge, cells=c, 
                point=p.InterestingPoint(edge=pedge, rule=MIDDLE))

    part_midpt(nri+1, sr)
    part_midpt(2*(nri+1), sr/2.0)

    for j in range(1, nri+1):
        e1 = p.edges
        z = j*sr
        pedge = e1[e1.findAt(((tw_fb/2.0, 0.0, z),))[0].index]
        p.PartitionCellByPlanePointNormal(normal=pedge, cells=c, 
            point=p.InterestingPoint(edge=pedge, rule=MIDDLE))

    p = mdb.models['Model-1'].parts['FB_Inner']
    c = p.cells
    pickedCells = c
    e, v2, d2 = p.edges, p.vertices, p.datums
    

    for i in range(1,(nri/2)+1):
        theNormalEdge = e.findAt(((-(tw_fb/2), dfb, (B-3*sr)/2),))
        p.PartitionCellByPlanePointNormal(point=(0,0,i*sr-(tr/2)-rch-rcu), normal=theNormalEdge[0], cells=pickedCells)
        theNormalEdge = e.findAt(((-(tw_fb/2), dfb, (B-3*sr)/2),))
        p.PartitionCellByPlanePointNormal(point=(0,0,i*sr+(tr/2)+rch+rcu), normal=theNormalEdge[0], cells=pickedCells)
    


def FBhorizontalPartitions():
    import part
    import sketch

    m = mdb.models['Model-1']
    p = m.parts['FB_Inner']
    c = p.cells
    pickedCells = c
    e, v2, d2 = p.edges, p.vertices, p.datums
    
    theNormalEdge = e.findAt(((-(tw_fb/2), dfb/2, 0),))
    p.PartitionCellByPlanePointNormal(point=(0,dfb-sr+wr+(boxLength/2),0), normal=theNormalEdge[0], cells=pickedCells)


    theNormalEdge = e.findAt(((-(tw_fb/2), dfb/2, 0),))
    p.PartitionCellByPlanePointNormal(point=(0,dfb-sr+wr,0), normal=theNormalEdge[0], cells=pickedCells)


    theNormalEdge = e.findAt(((-(tw_fb/2), dfb/2, 0),))
    p.PartitionCellByPlanePointNormal(point=(0,dfb-sr+wr-(boxLength/2),0), normal=theNormalEdge[0], cells=pickedCells)


    theNormalEdge = e.findAt(((-(tw_fb/2), dfb/2, 0),))
    p.PartitionCellByPlanePointNormal(point=(0,0,0), normal=theNormalEdge[0], cells=pickedCells)

def createRib():
    import part
    import sketch
    s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=200.0)
    g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
    s1.setPrimaryObject(option=STANDALONE)
    s1.rectangle(point1=(0.0, 0.0), point2=(tr, hr))
    p = mdb.models['Model-1'].Part(name='RibAS', dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p = mdb.models['Model-1'].parts['RibAS']
    p.BaseSolidExtrude(sketch=s1, depth=L)
    s1.unsetPrimaryObject()
    p = mdb.models['Model-1'].parts['RibAS']
    del mdb.models['Model-1'].sketches['__profile__']
    
def partitionRibAS():
    import part
    import sketch

    p = mdb.models['Model-1'].parts['RibAS']
    c = p.cells
    pickedCells = c
    e, v2, d2 = p.edges, p.vertices, p.datums
    
    theNormalEdge = e.findAt(((0, 0, L/2),))
    
    p.PartitionCellByPlanePointNormal(point=(0,0,lp-(tw_fb/2)), normal=theNormalEdge[0], cells=pickedCells)
    p = mdb.models['Model-1'].parts['RibAS']
    c = p.cells
    e1, v, d1 = p.edges, p.vertices, p.datums
    
    theNormalEdge1 = e1.findAt(((0, 0, L/2),))
    
    p.PartitionCellByPlanePointNormal(point=(0.0, 0.0, lp+(tw_fb/2)), normal=theNormalEdge1[0], cells=pickedCells)
    p = mdb.models['Model-1'].parts['RibAS']
    c = p.cells
    e, v2, d2 = p.edges, p.vertices, p.datums

    theNormalEdge = e.findAt(((0, 0, L/2),))

    p.PartitionCellByPlanePointNormal(point=(0.0, 0.0, (lp+s_fb)-(tw_fb/2)), normal=theNormalEdge[0], cells=pickedCells)
    p = mdb.models['Model-1'].parts['RibAS']
    c = p.cells
    e1, v, d1 = p.edges, p.vertices, p.datums

    theNormalEdge1 = e1.findAt(((0, 0, L/2),))

    p.PartitionCellByPlanePointNormal(point=(0.0, 0.0, (lp+s_fb)+(tw_fb/2)), normal=theNormalEdge1[0], 
        cells=pickedCells)
    p = mdb.models['Model-1'].parts['RibAS']
    c = p.cells
    e, v2, d2 = p.edges, p.vertices, p.datums

    theNormalEdge = e.findAt(((0, 0, L/2),))
    
    p.PartitionCellByPlanePointNormal(point=(0.0, 0.0, (lp+2*s_fb)-(tw_fb/2)), normal=theNormalEdge[0], cells=pickedCells)
    p = mdb.models['Model-1'].parts['RibAS']
    c = p.cells

    theNormalEdge1 = e1.findAt(((0, 0, L/2),))

    e1, v, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(point=(0.0, 0.0, (lp+2*s_fb)+(tw_fb/2)), normal=theNormalEdge1[0], 
        cells=pickedCells)
   

deckPlate();
partitionDeck();

create_innerFB();
create_AppleCutout_FBI();
create_CircleCutout_FBI();

create_outerFB(p_name = 'FB_Outer_A');
create_outerFB_C();

Girder();
GirderPartition();

createRib();
partitionRibAS();

FBverticalPartitions();
FBhorizontalPartitions();
