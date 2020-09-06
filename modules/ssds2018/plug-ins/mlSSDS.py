# -*- coding: utf-8 -*-
from maya import cmds
from maya.api import OpenMaya as om
from maya import OpenMayaUI as omui
import ssds
import ssds_ui
from PySide2.QtCore import * 
from PySide2.QtGui import *
from PySide2.QtWidgets import *
from PySide2.QtUiTools import *
import shiboken2 as shiboken

def maya_useNewAPI(): pass

RELEASE_DATE = '2020.9.6'
ssdsUiWindow = None


class ssdsUI(QMainWindow):
    ptr = omui.MQtUtil.mainWindow()
    parent = shiboken.wrapInstance(long(ptr), QWidget)
    titleName = 'SSDS v.' + RELEASE_DATE

    def __init__(self, parent = None):
        super(ssdsUI, self).__init__(self.parent)
        self.ui = ssds_ui.Ui_Dialog()
        self.ui.setupUi(self)
        self.setWindowTitle(self.titleName)
        self.model = QStringListModel()
        self.ui.choice.setModel(self.model)
        self.model.setStringList([])
        
    def invokeBuild(self):
        influences    = self.ui.spinInfluences.value()
        minNumJoints  = self.ui.spinMinNumJoints.value()
        maxNumJoints  = self.ui.spinMaxNumJoints.value()
        numIterations = self.ui.spinIterations.value()
        smoothness    = self.ui.spinLocality.value()
        numRings      = 0 if self.ui.radioRing0.isChecked() else (1 if self.ui.radioRing1.isChecked() else 2)
        uniformSample = self.ui.checkUniform.isChecked()

        transformTypes = []
        if self.ui.checkT.isChecked():
            transformTypes.append(0)
        if self.ui.checkTR.isChecked():
            transformTypes.append(1)
        if self.ui.checkTRS.isChecked():
            transformTypes.append(2)

        cmds.undoInfo(openChunk = True)
        try:
            cands = ssds.build(minNumJoints = minNumJoints,
                       maxNumJoints = maxNumJoints,
                       transformTypes = transformTypes,
                       numMaxInfluences = influences,
                       numIterations = numIterations,
                       smoothness = smoothness,
                       numRings = numRings,
                       uniformSample = uniformSample)
            if cands is not None:
                self.model.setStringList(cands)
                self.ui.choice.setCurrentIndex(self.model.createIndex(len(cands) - 1, 0))
        except Exception as e:
            raise e
        finally:
            cmds.undoInfo(closeChunk = True)

    def invokeMinNumChanged(self):
        minNumJoints = self.ui.spinMinNumJoints.value()
        maxNumJoints = self.ui.spinMaxNumJoints.value()
        self.ui.spinMaxNumJoints.setValue(max(minNumJoints, maxNumJoints))

    def invokeMaxNumChanged(self):
        minNumJoints = self.ui.spinMinNumJoints.value()
        maxNumJoints = self.ui.spinMaxNumJoints.value()
        self.ui.spinMinNumJoints.setValue(min(minNumJoints, maxNumJoints))

    def invokeSelectChoice(self, modelIndex):
        cmds.undoInfo(openChunk = True)
        try:
            ssds.select(modelIndex.row())
        except Exception as e:
            raise e
        finally:
            cmds.undoInfo(closeChunk = True)


def showUI(arg):
    global ssdsUiWindow
    if ssdsUiWindow == None:
        ssdsUiWindow = ssdsUI()
    ssdsUiWindow.show()


def initializePlugin(plugin):
    fnPlugin = om.MFnPlugin(plugin, vendor = 'Mukai Lab.', version = 'v.' + RELEASE_DATE)
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
    cmds.menuItem('SSDS', label = 'SSDS', command = showUI)


def deleteUI():
    try:
        cmds.deleteUI('SSDS', menuItem = True)
    except: pass
    try:
        itemArray = cmds.menu('MukaiLab', query = True, itemArray = True)
        if itemArray == None:
            cmds.deleteUI('MukaiLab')
    except: pass


