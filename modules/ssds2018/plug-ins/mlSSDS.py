# -*- coding: utf-8 -*-
from maya import cmds
from maya.api import OpenMaya as om
import ssds

def maya_useNewAPI(): pass


def initializePlugin(plugin):
    fnPlugin = om.MFnPlugin(plugin, vendor = 'Mukai Lab.', version = 'v.2018.10.27')
    try:
        createUI()
    except: raise
 
    
def uninitializePlugin(plugin):
    fnPlugin = om.MFnPlugin(plugin)
    try:
        deleteUI()
    except: raise


def createUI():
    cmds.setParent('MayaWindow')
    try:
        cmds.menu('MukaiLab', query = True, label = True)
    except:
        cmds.menu('MukaiLab', label = 'MukaiLab')
    cmds.setParent('MukaiLab', menu = True)
    cmds.menuItem('SSDS', label = 'SSDS', command = showBuildWindow)


def deleteUI():
    try:
        cmds.deleteUI('SSDS', menuItem = True)
    except: pass
    try:
        itemArray = cmds.menu('MukaiLab', query = True, itemArray = True)
        if itemArray == None:
            cmds.deleteUI('MukaiLab')
    except: pass


uiWindowName = 'SsdsWindow'
uiFormName = 'SsdsWindowForm'
uiFormLayoutName = 'SsdsFormLayout'
uiBuildButtonName = 'SsdsBuildButton'
uiMaxInfluenceName = ('SsdsMaxInfluenceLayout', 'SsdsMaxInfluenceField')
uiNumJointsName = ('SsdsNumJointsLayout', 'SsdsNumJointsField')
uiNumIterationsName = ('SsdsNumIterationsLayout', 'SsdsNumIterationsField')
uiTransformRadioCollectionName = 'SsdsTransformRadioCollection'
uiTransformNames = ('SsdsTransformT', 'SsdsTransformRT', 'SsdsTransformSRT')


def invokeBuild(arg):
    maxInfluence = cmds.intField(uiMaxInfluenceName[1], query = True, value = True)
    numJoints = cmds.intField(uiNumJointsName[1], query = True, value = True)
    numIterations = cmds.intField(uiNumIterationsName[1], query = True, value = True)
    transformStr = cmds.radioCollection(uiTransformRadioCollectionName, query = True, select = True)
    transformType = uiTransformNames.index(transformStr)

    cmds.undoInfo(openChunk = True)
    try:
        ssds.build(numJoints = numJoints,
                    transformType = transformType,
                    numMaxInfluences = maxInfluence,
                    numIterations = numIterations)
    except Exception as e:
        raise e
    finally:
        cmds.undoInfo(closeChunk = True)


def showBuildWindow(arg):
    labelWidth = 100
    fieldWidth = 100

    cmds.window(uiWindowName, title = 'SSDS')
    cmds.formLayout(uiFormName)
    cmds.columnLayout(uiFormLayoutName, rowSpacing = 5)

    # joints
    cmds.rowLayout(uiNumJointsName[0], numberOfColumns = 2,
                   columnWidth2 = (labelWidth, fieldWidth),
                   columnAlign2 = ('right', 'right'))
    cmds.text(label = '# Joints')
    cmds.intField(uiNumJointsName[1], minValue = 0, maxValue = 100, value = 0, width = fieldWidth)
    cmds.setParent('..')

    # max influences
    cmds.rowLayout(uiMaxInfluenceName[0], numberOfColumns = 2,
                   columnWidth2 = (labelWidth, fieldWidth),
                   columnAlign2 = ('right', 'right'))
    cmds.text(label = 'Max Influences')
    cmds.intField(uiMaxInfluenceName[1], minValue = 1, maxValue = 8, value = 4, width = fieldWidth)
    cmds.setParent('..')

    # iterations
    cmds.rowLayout(uiNumIterationsName[0], numberOfColumns = 2,
                   columnWidth2 = (labelWidth, fieldWidth),
                   columnAlign2 = ('right', 'right'))
    cmds.text(label = '# Iterations')
    cmds.intField(uiNumIterationsName[1], minValue = 0, maxValue = 100, value = 10, width = fieldWidth)
    cmds.setParent('..')

    # transform type
    cmds.rowLayout('SsdsTransformTypeLayout', numberOfColumns = 2,
                   columnWidth2 = (labelWidth, fieldWidth),
                   columnAlign2 = ('right', 'right'))
    cmds.text(label = 'Transform Type')
    cmds.columnLayout('temporary', rowSpacing = 3)
    cmds.radioCollection(uiTransformRadioCollectionName)
    cmds.radioButton(uiTransformNames[0], label = 'T')
    cmds.radioButton(uiTransformNames[1], label = 'R+T')
    cmds.radioButton(uiTransformNames[2], label = 'S+R+T')
    cmds.radioCollection(uiTransformRadioCollectionName, edit = True, select = uiTransformNames[2])
    cmds.setParent(uiFormLayoutName)

    # build
    cmds.button(uiBuildButtonName, label='Build', command = invokeBuild, width = labelWidth + fieldWidth)
    
    cmds.formLayout(uiFormName, edit = True,
                    attachForm = [(uiFormLayoutName, 'top', 5),
                                  (uiFormLayoutName, 'left', 5),
                                  (uiFormLayoutName, 'right', 5)])
    cmds.showWindow(uiWindowName)
