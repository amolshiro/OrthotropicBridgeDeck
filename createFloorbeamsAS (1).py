### List of parameters for developing the deck model
## Basic parameters
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
a=12

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


def polyline_sketch(m, pts):
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

def createFBinside():
    import part
    import sketch
    s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=200.0)
    s1.setPrimaryObject(option=STANDALONE)
    s1.Line(point1=(-tw_fb/2, dfb), point2=(tw_fb/2, dfb))
    s1.Line(point1=(tw_fb/2, dfb), point2=(tw_fb/2, 0.0))
    s1.Line(point1=(tw_fb/2, 0.0), point2=(flangeWidth/2, 0.0))
    s1.Line(point1=(flangeWidth/2, 0.0), point2=(flangeWidth/2, -tf_fb))
    s1.Line(point1=(flangeWidth/2, -tf_fb), point2=(-flangeWidth/2, -tf_fb))
    s1.Line(point1=(-flangeWidth/2, -tf_fb), point2=(-flangeWidth/2, 0.0))
    s1.Line(point1=(-flangeWidth/2, 0.0), point2=(-tw_fb/2, 0.0))
    s1.Line(point1=(-tw_fb/2, 0.0), point2=(-tw_fb/2, dfb))
    p = mdb.models['Model-1'].Part(name='FBInside', dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p = mdb.models['Model-1'].parts['FBInside']
    p.BaseSolidExtrude(sketch=s1, depth=(sg))
    s1.unsetPrimaryObject()
    p = mdb.models['Model-1'].parts['FBInside']


def AppleCutSketch():
    import part
    import sketch
    
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


    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
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

    mdb.models['Model-1'].sketches.changeKey(fromName='__profile__', 
        toName='AppleCutSketch')
    s.unsetPrimaryObject()



def createAppleCutoutArray():
    import part
    import sketch
    p = mdb.models['Model-1'].parts['FBInside']
    f1, e1 = p.faces, p.edges
    '''
    implement findAt
    '''
    frontalPlane = f1.findAt(((tw_fb/2.0, dfb/2.0, (sg-tw)/2.0),))
    rightmostEdge = e1.findAt(((tw_fb/2.0, dfb/2.0, 0.0),))
    t = p.MakeSketchTransform(sketchPlane=frontalPlane[0], sketchUpEdge=rightmostEdge[0], 
        sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(tw_fb/2, dfb/2, 
        s_fb))
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=294.18, gridSpacing=7.35, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=SUPERIMPOSE)
    p = mdb.models['Model-1'].parts['FBInside']
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)

    '''
    Revise for object list
    '''
    g = s.geometry
    old_geom_id = g.keys()
    s.retrieveSketch(sketch=mdb.models['Model-1'].sketches['AppleCutSketch'])
    g = s.geometry
    new_geom_id = g.keys()
    add_geom_id = list(set(new_geom_id) - set(old_geom_id))
    objList = tuple([g[i] for i in add_geom_id])
    s.move(vector=((sg/2)+(sr/2), dfb/2), objectList=objList)

    '''
    parameterize spacing
    '''
    s.linearPattern(geomList=objList, vertexList=(), number1=4, spacing1=sr, 
        angle1=180.0, number2=1, spacing2=1, angle2=90.0)
    p = mdb.models['Model-1'].parts['FBInside']
    f, e = p.faces, p.edges
    
    frontalPlane = f.findAt(((tw_fb/2.0, dfb/2.0, (sg-tw)/2.0),))
    rightmostEdge = e.findAt(((tw_fb/2.0, dfb/2.0, 0.0),))
    p.CutExtrude(sketchPlane=frontalPlane[0], sketchUpEdge=rightmostEdge[0], sketchPlaneSide=SIDE1, 
        sketchOrientation=RIGHT, sketch=s, flipExtrudeDirection=OFF)
    s.unsetPrimaryObject()


def CircleCutSketch():
    import part
    import sketch


    x1 = -tr/2 ; y1 = -(hr-rc)
    x2 = x1; y2 = cc
    x3 = tr/2 ; y3=y2
    x4 = x3; y4 = y1
    
    s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=200.0)
    g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
    s1.setPrimaryObject(option=STANDALONE)

    s1.Line(point1=(x1, y1), point2=(x2, y2))
    s1.Line(point1=(x2, y2), point2=(x3, y3))
    s1.Line(point1=(x3, y3), point2=(x4, y4))
    s1.ArcByCenterEnds(center=(cc, -hr), point1=(-tr/2, -(hr-rc)), point2=(tr/2, 
        -(hr-rc)), direction=COUNTERCLOCKWISE)
    mdb.models['Model-1'].sketches.changeKey(fromName='__profile__', 
        toName='Circlecutsketch')
    s1.unsetPrimaryObject()


def createCircleCutoutArray():
    import part
    import sketch
    p = mdb.models['Model-1'].parts['FBInside']
    f, e = p.faces, p.edges
    frontPlane = f.findAt(((tw_fb/2.0, dfb/2.0, (sg-tw)/2.0),))
    rightEdge = e.findAt(((tw_fb/2.0, dfb/2.0, 0.0),))
    t = p.MakeSketchTransform(sketchPlane=frontPlane[0], sketchUpEdge=rightEdge[0], 
        sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(-tw_fb/2, 
        dfb/2, s_fb))
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=294.18, gridSpacing=7.35, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=SUPERIMPOSE)
    p = mdb.models['Model-1'].parts['FBInside']
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
        number1=4, spacing1=sr, angle1=0.0, number2=1, spacing2=1, 
        angle2=90.0)
    p = mdb.models['Model-1'].parts['FBInside']
    f1, e1 = p.faces, p.edges
    frontPlane = f1.findAt(((tw_fb/2.0, dfb/2.0, (sg-tw)/2.0),))
    rightEdge = e1.findAt(((tw_fb/2.0, dfb/2.0, 0.0),))
    
    p.CutExtrude(sketchPlane=frontPlane[0], sketchUpEdge=rightEdge[0], sketchPlaneSide=SIDE1, 
        sketchOrientation=RIGHT, sketch=s, flipExtrudeDirection=OFF)
    s.unsetPrimaryObject()

def createSketches():
    AppleCutSketch();
    CircleCutSketch();

def completeFBInside():
    import part
    import sketch
    createSketches();
    createFBinside();
    createAppleCutoutArray();
    createCircleCutoutArray()


def createFBOutsideApple():
    import part
    import sketch
    s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=200.0)
    s1.setPrimaryObject(option=STANDALONE)
    s1.Line(point1=(-tw_fb/2, dfb), point2=(tw_fb/2, dfb))
    s1.Line(point1=(tw_fb/2, dfb), point2=(tw_fb/2, 0.0))
    s1.Line(point1=(tw_fb/2, 0.0), point2=(flangeWidth/2, 0.0))
    s1.Line(point1=(flangeWidth/2, 0.0), point2=(flangeWidth/2, -tf_fb))
    s1.Line(point1=(flangeWidth/2, -tf_fb), point2=(-flangeWidth/2, -tf_fb))
    s1.Line(point1=(-flangeWidth/2, -tf_fb), point2=(-flangeWidth/2, 0.0))
    s1.Line(point1=(-flangeWidth/2, 0.0), point2=(-tw_fb/2, 0.0))
    s1.Line(point1=(-tw_fb/2, 0.0), point2=(-tw_fb/2, dfb))
    p = mdb.models['Model-1'].Part(name='FBOutsideApple', dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p = mdb.models['Model-1'].parts['FBOutsideApple']
    p.BaseSolidExtrude(sketch=s1, depth=(sr+(sr/2)))
    s1.unsetPrimaryObject()
    p = mdb.models['Model-1'].parts['FBOutsideApple']

def FBOutsideAppleCutout():
    import part
    import sketch
    p = mdb.models['Model-1'].parts['FBOutsideApple']
    f1, e1 = p.faces, p.edges
    frontalPlane = f1.findAt(((tw_fb/2.0, dfb/2.0, tw/2.0),))
    rightmostEdge = e1.findAt(((tw_fb/2.0, dfb/2.0, 0.0),))
    t = p.MakeSketchTransform(sketchPlane=frontalPlane[0], sketchUpEdge=rightmostEdge[0], 
        sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(tw_fb/2, dfb/2, 
        sr-(sr/4)))
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=156.0, gridSpacing=3.9, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=SUPERIMPOSE)
    p = mdb.models['Model-1'].parts['FBOutsideApple']
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    old_geom_id = g.keys()
    s.retrieveSketch(sketch=mdb.models['Model-1'].sketches['AppleCutSketch'])
    new_geom_id = g.keys()
    add_geom_id = list(set(new_geom_id) - set(old_geom_id))
    objList = tuple([g[i] for i in add_geom_id])
    s.move(vector=(sr/4, dfb/2), objectList=objList)
    p = mdb.models['Model-1'].parts['FBOutsideApple']
    f, e = p.faces, p.edges
    p.CutExtrude(sketchPlane=frontalPlane[0], sketchUpEdge=rightmostEdge[0], sketchPlaneSide=SIDE1, 
        sketchOrientation=RIGHT, sketch=s, flipExtrudeDirection=OFF)
    s.unsetPrimaryObject()

def createFBOutsideCircle():
    import part
    import sketch
    s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=200.0)
    s1.setPrimaryObject(option=STANDALONE)
    s1.Line(point1=(-tw_fb/2, dfb), point2=(tw_fb/2, dfb))
    s1.Line(point1=(tw_fb/2, dfb), point2=(tw_fb/2, 0.0))
    s1.Line(point1=(tw_fb/2, 0.0), point2=(flangeWidth/2, 0.0))
    s1.Line(point1=(flangeWidth/2, 0.0), point2=(flangeWidth/2, -tf_fb))
    s1.Line(point1=(flangeWidth/2, -tf_fb), point2=(-flangeWidth/2, -tf_fb))
    s1.Line(point1=(-flangeWidth/2, -tf_fb), point2=(-flangeWidth/2, 0.0))
    s1.Line(point1=(-flangeWidth/2, 0.0), point2=(-tw_fb/2, 0.0))
    s1.Line(point1=(-tw_fb/2, 0.0), point2=(-tw_fb/2, dfb))
    p = mdb.models['Model-1'].Part(name='FBOutsideCircle', dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p = mdb.models['Model-1'].parts['FBOutsideCircle']
    p.BaseSolidExtrude(sketch=s1, depth=(sr+(sr/2)))
    s1.unsetPrimaryObject()
    p = mdb.models['Model-1'].parts['FBOutsideCircle']

def FBOutsideCircleCutout():
    import part
    import sketch
    p = mdb.models['Model-1'].parts['FBOutsideCircle']
    f1, e1 = p.faces, p.edges
    frontalPlane = f1.findAt(((tw_fb/2.0, dfb/2.0, tw/2.0),))
    rightmostEdge = e1.findAt(((tw_fb/2.0, dfb/2.0, 0.0),))
    t = p.MakeSketchTransform(sketchPlane=frontalPlane[0], sketchUpEdge=rightmostEdge[0], 
        sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(tw_fb/2, dfb/2, 
        sr-(sr/4)))
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=156.0, gridSpacing=3.9, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=SUPERIMPOSE)
    p = mdb.models['Model-1'].parts['FBOutsideCircle']
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    old_geom_id = g.keys()
    s.retrieveSketch(sketch=mdb.models['Model-1'].sketches['Circlecutsketch'])
    new_geom_id = g.keys()
    add_geom_id = list(set(new_geom_id) - set(old_geom_id))
    objList = tuple([g[i] for i in add_geom_id])
    s.move(vector=(-sr/4, dfb/2), objectList=objList)
    p = mdb.models['Model-1'].parts['FBOutsideCircle']
    f, e = p.faces, p.edges
    p.CutExtrude(sketchPlane=frontalPlane[0], sketchUpEdge=rightmostEdge[0], sketchPlaneSide=SIDE1, 
        sketchOrientation=RIGHT, sketch=s, flipExtrudeDirection=OFF)
    s.unsetPrimaryObject()

def completeFBOutside():
    createFBOutsideApple();
    FBOutsideAppleCutout();
    createFBOutsideCircle();
    FBOutsideCircleCutout();

def createFloorbeams():
    completeFBInside();
    completeFBOutside();
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
def deckPlate():

    pts = [(0,0),
           (L,0),
           (L,B),
           (0,B),
           (0,0)]

    m = mdb.models['Model-1']
    s1 = polyline_sketch(m,pts)
    p = mdb.models['Model-1'].Part(name='deckPlate', dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p.BaseSolidExtrude(sketch=s1, depth=t_dp)
    s1.unsetPrimaryObject()
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
            p.PartitionCellByPlaneThreePoints(point1=(0,0,lp+sfb*n), point2=(L,0,lp+sfb*n),cells=pickedCells, point3=(0,L,lp+sfb*n))
            p.PartitionCellByPlaneThreePoints(point1=(0,0,lp+tw_fb+sfb*n), point2=(L,0,lp+tw_fb+sfb*n),cells=pickedCells, point3=(0,L,tw_fb+lp+sfb*n))
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
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, 
        elemType3))
    p.generateMesh()
def partitionDeck():
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
    
    p = mdb.models['Model-1'].parts['deckPlate']
    c = p.cells
    cells = c
    pickedCells = c
    n = 0
    while(n<nfb):
        c = p.cells
        cells = c
        pickedCells = c

        p.PartitionCellByPlaneThreePoints(point1=(lp-tw_fb*0.5+sfb*n,0,0), point2=(lp-tw_fb*0.5+sfb*n,0,L),cells=pickedCells, point3=(lp-tw_fb*0.5+sfb*n,L,0))

        c = p.cells
        cells = c
        pickedCells = c

        p.PartitionCellByPlaneThreePoints(point1=(lp+tw_fb*0.5+sfb*n,0,0), point2=(lp+tw_fb*0.5+sfb*n,0,L),cells=pickedCells, point3=(lp+tw_fb*0.5+sfb*n,L,0))
        n = n+1
    n = 0
    
    pickedCells = c
    p.PartitionCellByPlaneThreePoints(point1=(0,tp-tr*0.5,0), point2=(0,tp-tr*0.5,L),cells=pickedCells, point3=(L,tp-tr*0.5,0))
    pickedCells = c
    p.PartitionCellByPlaneThreePoints(point1=(0,tp+tr*0.5,0), point2=(0,tp+tr*0.5,L),cells=pickedCells, point3=(L,tp+tr*0.5,0))
    
    pickedCells = c
    p.PartitionCellByPlaneThreePoints(point1=(0,B-(tp-tr*0.5),0), point2=(0,B-(tp-tr*0.5),L),cells=pickedCells, point3=(L,B-(tp-tr*0.5),0))
    pickedCells = c
    p.PartitionCellByPlaneThreePoints(point1=(0,B-(tp+tr*0.5),0), point2=(0,B-(tp+tr*0.5),L),cells=pickedCells, point3=(L,B-(tp+tr*0.5),0))    
    
    
    ##Girded cut
    
    while(n < ng):
        pickedCells = c
        p.PartitionCellByPlaneThreePoints(point1=(0,tp+sr-tw*0.5+n*sr*(nri+1),0), point2=(0,tp+sr-tw*0.5+n*sr*(nri+1),L),cells=pickedCells, point3=(L,tp+sr-tw*0.5+n*sr*(nri+1),0))
        p.PartitionCellByPlaneThreePoints(point1=(0,tp+sr+tw*0.5+n*sr*(nri+1),0), point2=(0,tp+sr+tw*0.5+n*sr*(nri+1),L),cells=pickedCells, point3=(L,tp+sr+tw*0.5+n*sr*(nri+1),0))
        n = n+1
    
        
    n = 0 
    while(n<nri):
        pickedCells = c
        p.PartitionCellByPlaneThreePoints(point1=(0,sr+sr+tp-tr*0.5+sr*n,0), point2=(0,sr+sr+tp-tr*0.5+sr*n,L),cells=pickedCells, point3=(L,sr+sr+tp-tr*0.5+sr*n,0))
        p.PartitionCellByPlaneThreePoints(point1=(0,sr+sr+tp+tr*0.5+sr*n,0), point2=(0,sr+sr+tp+tr*0.5+sr*n,L),cells=pickedCells, point3=(L,sr+sr+tp+tr*0.5+sr*n,0))
        n = n+1
 
    p = mdb.models['Model-1'].parts['deckPlate']
    p.seedPart(size=1.0, deviationFactor=0.1, minSizeFactor=0.1)
   
    elemType1 = mesh.ElemType(elemCode=C3D20R, elemLibrary=STANDARD)
    elemType2 = mesh.ElemType(elemCode=C3D15, elemLibrary=STANDARD)
    elemType3 = mesh.ElemType(elemCode=C3D10, elemLibrary=STANDARD)
    cells = c#.getSequenceFromMask(mask=('[#ffffffff #3 ]', ), ) ##use find at, create container
    pickedRegions =(cells, )
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, 
        elemType3))
    p.generateMesh()

def createRib():
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

def partitionTest():
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
    #horizontalNormal = e.findAt(((tw_fb/2.0, dfb/2.0, 0.0),))
    verticalNormal = e.findAt(((tw_fb/2, dfb, 30),))
    p.PartitionCellByPlanePointNormal(normal=verticalNormal[0], cells=pickedCells, 
        point=p.InterestingPoint(edge=verticalNormal[0], rule=MIDDLE))

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
    
def partitionFBInside():
    verticalPartitions();
    horizontalPartitions();

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

def FBOutsidePartitionApple():
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
    p = mdb.models['Model-1'].parts['FBOutsideApple']
    p.DatumPointByCoordinate(coords=(0.25, 21.917326, 18.0))
    p = mdb.models['Model-1'].parts['FBOutsideApple']
    p.DatumPointByCoordinate(coords=(0.25, 14.311399, 18.0))
    p = mdb.models['Model-1'].parts['FBOutsideApple']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
    e1, v2, d2 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[36], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[36], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBOutsideApple']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#3 ]', ), )
    e, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(point=d1[3], normal=e[17], cells=pickedCells)
    p = mdb.models['Model-1'].parts['FBOutsideApple']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#11 ]', ), )
    e1, v2, d2 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(point=d2[4], normal=e1[17], 
        cells=pickedCells)
    p = mdb.models['Model-1'].parts['FBOutsideApple']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#3 ]', ), )
    e, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[1], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[83], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBOutsideApple']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#13 ]', ), )
    e1, v2, d2 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(point=v2[69], normal=e1[63], 
        cells=pickedCells)
    p = mdb.models['Model-1'].parts['FBOutsideApple']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#2a ]', ), )
    e, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(point=v1[21], normal=e[22], 
        cells=pickedCells)
    p = mdb.models['Model-1'].parts['FBOutsideApple']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#e00 ]', ), )
    e1, v2, d2 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[99], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[100], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBOutsideApple']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#15050 ]', ), )
    e, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[60], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[153], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBOutsideApple']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#510a00 ]', ), )
    e1, v2, d2 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[115], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[135], rule=MIDDLE))

def FBOutsidePartCircle():
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
    p = mdb.models['Model-1'].parts['FBOutsideCircle']
    p.DatumPointByCoordinate(coords=(0.25, 21.917326, 18.0))
    p = mdb.models['Model-1'].parts['FBOutsideCircle']
    p.DatumPointByCoordinate(coords=(0.25, 18.25, 18.0))
    p = mdb.models['Model-1'].parts['FBOutsideCircle']
    p.DatumPointByCoordinate(coords=(0.25, 14.311399, 18.0))
    p = mdb.models['Model-1'].parts['FBOutsideCircle']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
    e, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[15], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[15], rule=MIDDLE))
    p = mdb.models['Model-1'].parts['FBOutsideCircle']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#3 ]', ), )
    e1, v2, d2 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(point=d2[4], normal=e1[21], 
        cells=pickedCells)
    p = mdb.models['Model-1'].parts['FBOutsideCircle']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#12 ]', ), )
    e, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(point=d1[5], normal=e[38], cells=pickedCells)
    p = mdb.models['Model-1'].parts['FBOutsideCircle']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#68 ]', ), )
    e1, v2, d2 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(point=d2[3], normal=e1[50], 
        cells=pickedCells)
    p = mdb.models['Model-1'].parts['FBOutsideCircle']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#140 ]', ), )
    e, v1, d1 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e[67], cells=pickedCells, 
        point=p.InterestingPoint(edge=e[75], rule=CENTER))
    p = mdb.models['Model-1'].parts['FBOutsideCircle']
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#284 ]', ), )
    e1, v2, d2 = p.edges, p.vertices, p.datums
    p.PartitionCellByPlanePointNormal(normal=e1[36], cells=pickedCells, 
        point=p.InterestingPoint(edge=e1[21], rule=MIDDLE))

def partitionFBOutside():
    FBOutsidePartitionApple();
    FBOutsidePartCircle();

Girder();
createFloorbeams();
deckPlate();
GirderPartition();
#partitionDeck();
createRib();
#partitionTest();
partitionFBInside();
#meshFBInside();
partitionFBOutside();
