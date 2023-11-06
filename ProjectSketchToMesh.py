#Author - Nick Otradovec
#Description - Creates a 3D Sketch Line or Lines Projected along Z onto a Mesh from a 2D Sketch

import adsk.core, adsk.fusion, traceback
import math
import time

from . import commands
from .lib import fusion360utils as futil

_des = adsk.fusion.Design.cast(None)
_activeSketch = adsk.fusion.Sketch.cast(None)
_meshSelectInput = adsk.core.SelectionCommandInput.cast(None)
_sketchSelectInput = adsk.core.SelectionCommandInput.cast(None)
_meshState = []
_controlId = 'sketchToMesh'

_maxSpacingInput = adsk.core.ValueCommandInput.cast(None)
_searchToleranceInput = adsk.core.ValueCommandInput.cast(None)

local_handlers = []

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        # Get the CommandDefinitions collection.
        cmdDef = ui.commandDefinitions.addButtonDefinition(_controlId, 'Project Sketch to Mesh',
                                                            'Project Sketch through Z onto Mesh',
                                                            './/Resources//MeshIntersect')
        futil.add_handler(cmdDef.commandCreated, MeshIntersectCommandCreatedEventHandler, local_handlers=local_handlers)
        
        sketchPanel = ui.allToolbarPanels.itemById('SketchPanel')
        projDropDown = sketchPanel.controls.itemById('ProjectIncludeDropDown')
        projDropDown.controls.addCommand(cmdDef, _controlId, True)

        adsk.autoTerminate(False)
        
    except:
        if ui:
            #ui.messageBox('Unexpected failure.', 'Intersect Mesh Body')
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def stop(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        sketchPanel = ui.allToolbarPanels.itemById('SketchPanel')
        projDropDown = sketchPanel.controls.itemById('ProjectIncludeDropDown')
        
        # Get the meshIntersect control.
        meshCntrl = projDropDown.controls.itemById(_controlId)
        if meshCntrl:
            meshCntrl.deleteMe()
        
        meshInterectCommandDef = ui.commandDefinitions.itemById(_controlId)
        if meshInterectCommandDef:
            meshInterectCommandDef.deleteMe()
 
        futil.clear_handlers()
        
    except:
        if ui:
            #ui.messageBox('Unexpected failure removing command.', 'Intersect Mesh Body')
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def MeshIntersectCommandCreatedEventHandler(args: adsk.core.CommandCreatedEventArgs):
    try:                     
        app = adsk.core.Application.get()
        ui  = app.userInterface           

        # Check that a design is active.
        global _des
        _des = app.activeProduct
        if not _des:
            ui.messageBox('A design must be active.')
            return
            
        # Check to see if a sketch is active and get it.
        if app.activeEditObject.objectType == adsk.fusion.Sketch.classType():
            sk = app.activeEditObject
            global _activeSketch
            _activeSketch = adsk.fusion.Sketch.cast(sk)

        cmd = adsk.core.Command.cast(args.command)
        cmd.isExecutedWhenPreEmpted = False
        inputs = cmd.commandInputs

        futil.add_handler(args.command.activate, CommandActivatedHandler, local_handlers=local_handlers)
        futil.add_handler(args.command.execute, MeshIntersectCommandExecutedEventHandler, local_handlers=local_handlers)
        #futil.add_handler(args.command.inputChanged, InputChangedHandler, local_handlers=local_handlers)
        futil.add_handler(args.command.destroy, DestroyHandler, local_handlers=local_handlers)

        # Create the input for selecting the mesh bodies.
        global _meshSelectInput
        _meshSelectInput = inputs.addSelectionInput('meshSelect', 'Mesh Bodies', 'Select mesh bodies to section.')
        _meshSelectInput.addSelectionFilter('MeshBodies')
        _meshSelectInput.setSelectionLimits(1, 0)

        # Create the input for selecting the sketch items.
        global _sketchSelectInput
        _sketchSelectInput = inputs.addSelectionInput('sketchSelect', 'Sketch Lines Included', 'Select sketch items to Include.')
        _sketchSelectInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchLines)
        #_sketchSelectInput.addSelectionFilter(adsk.core.SelectionCommandInput.SketchCurves)
        #_sketchSelectInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionLines)
        #_sketchSelectInput.addSelectionFilter(adsk.core.SelectionCommandInput.Sketches)
        _sketchSelectInput.setSelectionLimits(1)

        global _maxSpacingInput
        _maxSpacingInput = inputs.addDistanceValueCommandInput('spacingSelect', 'Line Point Spacing', adsk.core.ValueInput.createByReal(.5))
        
        global _searchToleranceInput
        _searchToleranceInput = inputs.addDistanceValueCommandInput('searchToleranceSelect', 'Search Tolerance Distance', adsk.core.ValueInput.createByReal(.1))             
    
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))   

def DestroyHandler(args: adsk.core.CommandEventArgs):
    futil.log(f'Command Destroy Event "{args.terminationReason}"')
    global local_handlers
    local_handlers = []
    
# Event handler for activate event.
def CommandActivatedHandler(args: adsk.core.CommandEventArgs):
    global _meshState
    _meshState.clear
    for comp in _des.allComponents:
        for mesh in comp.meshBodies:
            _meshState.append([mesh, mesh.isSelectable])
            if mesh.isSelectable == False:
                mesh.isSelectable = True
               
def MeshIntersectCommandExecutedEventHandler(args: adsk.core.CommandEventArgs):                    
    try: 
        app = adsk.core.Application.get()
        ui  = app.userInterface

        #https://stackoverflow.com/questions/3252194/numpy-and-line-intersections#answer-9110966
        #https://stackoverflow.com/review/suggested-edits/531949

        progDialog = ui.createProgressDialog()

        #TODO: Is post processing required to "toggle" the laser on and off?  
        #TODO: Is velocity adjustment necessary to get an appropriate effect laser speed on slanted surfaces?

        #TODO: Better algorithm here:
        #1: Build a dictionary of what faces are adjacent to other faces. (Must have two of the same points)
        #2. Find rough first point. Use below method to determine exactly where.
        #3. Moving "outwards" incrementally, evaluate all adjacent faces of the current "outline".
        #4. These become the new "outline" or search perimeter
        #5. Repeat until next point is found.       
        line = CalculateIntersection(_meshSelectInput.selection(0).entity, _activeSketch, progDialog)
        if line != None and not progDialog.wasCancelled:
            CreateLine(_activeSketch, line)
        
        progDialog.hide()
        
    except:
        if ui:
            if progDialog:
                progDialog.hide()
            ui.messageBox('Unexpected Execute failure:\n{}'.format(traceback.format_exc()))

def LineSegments(sketch):
    points = []
    
    threshhold_toLastPoint = .001
    priorPoint_X = -10000000
    proirPoint_Y = -10000000
    
    index_lineSequence = -1
    
    maxSpacing = _maxSpacingInput.value

    for i in range(0, _sketchSelectInput.selectionCount):
        obj = _sketchSelectInput.selection(i).entity      
        if obj.objectType == adsk.fusion.SketchLine.classType():
            
            line = adsk.fusion.SketchLine.cast(obj)
            
            if Distance(priorPoint_X, proirPoint_Y, line.startSketchPoint.worldGeometry.x, 
                        line.startSketchPoint.worldGeometry.y) > threshhold_toLastPoint:
                                       
                index_lineSequence = index_lineSequence + 1 # new set of points
                points.append([])

            pointCount = math.floor( line.length / maxSpacing )
            dx = (line.endSketchPoint.worldGeometry.x - line.startSketchPoint.worldGeometry.x) / pointCount
            dy = (line.endSketchPoint.worldGeometry.y - line.startSketchPoint.worldGeometry.y) / pointCount
            
            for j in range(0, pointCount):
                points[index_lineSequence].append([line.startSketchPoint.worldGeometry.x + j*dx, 
                                                   line.startSketchPoint.worldGeometry.y + j*dy, 
                                                   0]) # we'll set z according to the mesh later.
                
            priorPoint_X = line.endSketchPoint.worldGeometry.x
            proirPoint_Y = line.endSketchPoint.worldGeometry.y     
            
        elif obj.objectType == adsk.fusion.SketchCurve.classType():          
            raise Exception("SketchCurve not implemented.")
        elif obj.objectType == adsk.fusion.SketchArc.classType():
            raise Exception("SketchArc not implemented.")
        elif obj.objectType == adsk.fusion.Sketch.classType():
            raise Exception("Sketch not yet configured. Please select the individual composing lines/curves in the meantime.")
        else:
            raise Exception("Unexpected Sketch object" + obj.objectType)
        
    return points

# Returns loops of coordinates.
def CalculateIntersection(mesh, sketch: adsk.fusion.Sketch, progDialog: adsk.core.ProgressDialog):
        
    #1. Generate lists of XY values to evaluate based on the sketch lines.
    #   For a continuous line, there would be one list. How is this captured in the sketch object?
    #   Each line needs to be evaluate to determine the resolution and how many points
    #   should be created per distance
    points = LineSegments(sketch)
    
    progTotal = 0
    for idxLineSeg in range(0, len(points)):
        progTotal += len(points[idxLineSeg])
    
    progDialog.show('Projection Calculation', 'Evaluating Intersection: {} segments'.format(progTotal), 0, progTotal) 
    progDialog.progressValue = 0
    
    #2. For each XY value, evaluate what the Z coordinate of the mesh is
    triangleMesh = mesh.displayMesh
    nodeCoords = triangleMesh.nodeCoordinatesAsDouble
    nodeIndices = triangleMesh.nodeIndices
    
    searchTolerance = _searchToleranceInput.value
 
    for idxLineSeg in range(0, len(points)):
        for idxPoint in range(0, len(points[idxLineSeg])):
            
            progDialog.progressValue += 1
            if progDialog.wasCancelled: return []
            
            for i in range(0, len(nodeIndices)):
                
                x = nodeCoords[3*nodeIndices[i]]
                y = nodeCoords[(3*nodeIndices[i])+1]
                z = nodeCoords[(3*nodeIndices[i])+2]

                if abs(points[idxLineSeg][idxPoint][0] - x) < searchTolerance and abs(points[idxLineSeg][idxPoint][1] - y) < searchTolerance :
                
                # About half as fast, but we could use to use the "best" fit if desired.
                # Regardless, implementing the nearest neighbor hashing is really the better solution anyways.
                #if Distance(x, y, points[idxLineSeg][idxPoint][0], points[idxLineSeg][idxPoint][1]) < searchTolerance :                                       
                    points[idxLineSeg][idxPoint][2] = z
                    break
    
    return points        
 
def CreateLine(sketch: adsk.fusion.Sketch, points):
    
    sketch.isComputeDeferred = True
    lines = sketch.sketchCurves.sketchLines
        
    for idxLineSeg in range(0, len(points)):
        for idxPoint in range(1, len(points[idxLineSeg])):
            lines.addByTwoPoints(adsk.core.Point3D.create(points[idxLineSeg][idxPoint-1][0], points[idxLineSeg][idxPoint-1][1], points[idxLineSeg][idxPoint-1][2]), 
                                 adsk.core.Point3D.create(points[idxLineSeg][idxPoint][0], points[idxLineSeg][idxPoint][1], points[idxLineSeg][idxPoint][2])) 
               
    sketch.isComputeDeferred = False
    
def Distance(x1, y1, x2, y2):
    return math.sqrt(math.pow((x1-x2),2) + math.pow((y1-y2),2))