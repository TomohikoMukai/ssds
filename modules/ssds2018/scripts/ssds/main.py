# -*- coding: utf-8 -*-
from maya import cmds
from maya.api import OpenMaya as om
from maya.api import OpenMayaAnim as oma
import math
import numpy as np
import native


class SkinJoint:
    def __init__(self, path = None, name = '', bindPose = np.eye(4)):
        self.path = path
        self.name = name
        self.bindPose = bindPose.copy()

    def __str__(self):
        return self.name


def getMesh():
    slist = om.MGlobal.getActiveSelectionList()
    itsl = om.MItSelectionList(slist)
    meshPaths = []
    while not itsl.isDone():
        dagPath = itsl.getDagPath()
        itsl.next()
        if dagPath is None:
            continue
        apiType = dagPath.apiType()
        if apiType != om.MFn.kTransform:
            continue
        for c in xrange(dagPath.childCount()):
            child = dagPath.child(c)
            if child.apiType() != om.MFn.kMesh:
                continue
            path = dagPath.getAPathTo(child)
            mesh = om.MFnMesh(path)
            if not mesh.findPlug('intermediateObject', True).asBool():
                meshPaths.append(path)
                break
    return meshPaths


def concatenatePointLists(meshPaths):
    retval = np.empty([0, 3])
    for path in meshPaths:
        mesh = om.MFnMesh(path)
        points = mesh.getPoints(om.MSpace.kWorld)
        points = np.array([[p.x, p.y, p.z] for p in points])
        retval = np.append(retval, points.reshape(-1, 3), axis = 0)
    return retval


def concatenateNeighborLists(meshPaths):
    neighbor = []
    for path in meshPaths:
        mesh = om.MFnMesh(path)
        _, indices = mesh.getTriangles()
        offset = len(neighbor)
        neighbor = neighbor + [set() for v in xrange(mesh.numVertices)]
        for l in xrange(len(indices) / 3):
            i0 = indices[l * 3 + 0] + offset
            i1 = indices[l * 3 + 1] + offset
            i2 = indices[l * 3 + 2] + offset
            neighbor[i0].add(i1)
            neighbor[i0].add(i2)
            neighbor[i1].add(i0)
            neighbor[i1].add(i2)
            neighbor[i2].add(i0)
            neighbor[i2].add(i1)
    maxlen = 0
    for i in xrange(len(neighbor)):
        maxlen = max(maxlen, len(neighbor[i]))
    retval = -np.ones([len(neighbor), maxlen], dtype = np.longlong)
    for i in xrange(len(neighbor)):
        retval[i, 0:len(neighbor[i])] = list(neighbor[i])
    return retval


def bindToSkin(meshPaths, skinIndex, skinWeight,
              skinJnts, numMaxInfluences):
    asl = om.MSelectionList()
    asl.clear()
    jntNames = [sj.name for sj in skinJnts]
    for sj in skinJnts:
        m = om.MMatrix(sj.bindPose.tolist())
        m = om.MTransformationMatrix(m)
        om.MFnTransform(sj.path).setTransformation(m)
        asl.add(sj.path)
    offset = 0
    for meshPath in meshPaths:
        mesh = om.MFnMesh(meshPath)
        sl = om.MSelectionList(asl)
        sl.add(meshPath)
        om.MGlobal.setActiveSelectionList(sl)
        meshName = om.MFnDagNode(mesh.parent(0)).name()
        skinName = cmds.skinCluster(maximumInfluences = numMaxInfluences,
                                    name = meshName + 'Cluster',
                                    toSelectedBones = True)[0]
        skinObj = om.MGlobal.getSelectionListByName(skinName).getDependNode(0)
        skin = oma.MFnSkinCluster(skinObj)
        vertexIndices = om.MIntArray(mesh.numVertices, 0)
        for i in xrange(mesh.numVertices):
            vertexIndices[i] = i
        singleIndexedComp = om.MFnSingleIndexedComponent()
        vertexComp = singleIndexedComp.create(om.MFn.kMeshVertComponent)
        singleIndexedComp.addElements(vertexIndices)
        infDags = skin.influenceObjects()
        numInfDags = len(infDags)
        infIndices = om.MIntArray(numInfDags, 0)
        for i in xrange(numInfDags):
            infIndices[i] = i
        weights = om.MDoubleArray(mesh.numVertices * numInfDags, 0)
        for v in xrange(mesh.numVertices):
            for j, w in zip(skinIndex[offset + v], skinWeight[offset + v]):
                if j >= 0:
                    weights[v * numInfDags + j] = w
        skin.setWeights(meshPath, vertexComp, infIndices, weights)
        offset += mesh.numVertices 
        skin.findPlug('deformUserNormals', True).setBool(False)


def cloneMeshs(meshPaths):
    cloneMeshPaths = []
    cloneGroup = cmds.group(empty = True, world = True, name = 'ssdsResult')
    cloneGroupSL = om.MGlobal.getSelectionListByName(cloneGroup)
    cloneGroupFn = om.MFnTransform(cloneGroupSL.getDagPath(0))
    for path in meshPaths:
        mesh = om.MFnMesh(path)
        meshName = om.MFnDagNode(mesh.parent(0)).name()
        cloneMeshName = 'ssds:' + meshName
        sl = om.MSelectionList();
        sl.clear()
        sl.add(path)
        om.MGlobal.setActiveSelectionList(sl)
        cmds.duplicate(returnRootsOnly = True, name = cloneMeshName, renameChildren = True)
        cmds.parent(cloneMeshName, cloneGroup)
        cmds.setAttr(cloneMeshName + '.inheritsTransform', False)
        cloneMeshSL = om.MGlobal.getSelectionListByName(cloneMeshName)
        cloneMeshPath = cloneMeshSL.getDagPath(0)
        cloneMeshPath.extendToShape()
        cloneMeshPaths.append(cloneMeshPath)
    return cloneMeshPaths, cloneGroup


def sampleShapes(meshPaths):
    startTime = oma.MAnimControl.animationStartTime()
    endTime = oma.MAnimControl.animationEndTime()
    ctime = startTime
    oma.MAnimControl.setCurrentTime(ctime)
    pnts = concatenatePointLists(meshPaths)
    shapeSample = np.empty((0, len(pnts), 3))
    while ctime <= endTime:
        shapeSample = np.append(shapeSample, pnts.reshape(1, -1, 3), axis = 0)
        ctime = ctime + 1
        oma.MAnimControl.setCurrentTime(ctime)
        pnts = concatenatePointLists(meshPaths)
    oma.MAnimControl.setCurrentTime(startTime)
    return shapeSample


def bakeJointMotion(skinJoints, skinMatrix):
    asl = om.MSelectionList()
    for sj in skinJoints:
        asl.add(sj.path)
    om.MGlobal.setActiveSelectionList(asl)
    startTime = oma.MAnimControl.animationStartTime()
    endTime = oma.MAnimControl.animationEndTime()
    frame = 0
    ctime = startTime
    oma.MAnimControl.setCurrentTime(ctime)
    while ctime <= endTime:
        oma.MAnimControl.setCurrentTime(ctime)
        for jid, sj in enumerate(skinJoints):
            m = om.MMatrix(np.dot(sj.bindPose, skinMatrix[jid, frame]).tolist())
            m = om.MTransformationMatrix(m)
            om.MFnTransform(sj.path).setTransformation(m)
        cmds.setKeyframe()
        frame = frame + 1
        ctime = ctime + 1
    oma.MAnimControl.setCurrentTime(startTime)


def build(numJoints = 4,
          transformType = 2,
          numMaxInfluences = 4,
          numIterations = 5,
          concentrate = 1.0):
    srcMeshPaths = getMesh()
    if len(srcMeshPaths) == 0:
        raise Exception('Select mesh')
    srcMeshNames = []
    for p in srcMeshPaths:
        srcMeshNames.append(om.MFnMesh(p).name())
    om.MGlobal.displayInfo('SSDS 2019.1.21')
    om.MGlobal.displayInfo(' # joints: '     + str(numJoints))
    om.MGlobal.displayInfo(' # iterations: ' + str(numIterations))
    om.MGlobal.displayInfo(' transform: '    + str(transformType))
    om.MGlobal.displayInfo(' concentrate: '  + str(concentrate))
    if not cmds.namespace(exists = 'ssds'):
        cmds.namespace(add = 'ssds')   
    # data acquisition
    oma.MAnimControl.setCurrentTime(oma.MAnimControl.animationStartTime())
    shapeSample = sampleShapes(srcMeshPaths)
    initPos = shapeSample[0]
    numVertices = shapeSample.shape[1]
    neighborVertices = concatenateNeighborLists(srcMeshPaths)
    # initialize native module
    pinput, poutput = native.initialize(
        initPos, shapeSample, neighborVertices, numJoints, numMaxInfluences)
    # skinning decomposition
    numJoints = native.clusterVertices(pinput, poutput, transformType)
    for it in xrange(numIterations):
        om.MGlobal.displayInfo('Iteration #' + str(it + 1))
        native.updateSkinWeight(pinput, poutput, concentrate * 1.0e-5)
        native.updateJointTransform(pinput, poutput, transformType)
    # data retrieval from native module
    skinIndex = -np.ones([numVertices, numMaxInfluences], dtype = np.int)
    skinWeight = np.zeros([numVertices, numMaxInfluences])
    skinMatrix = np.zeros([numJoints, shapeSample.shape[0], 4, 4])
    clusterCenter = np.zeros([numJoints, 3])
    native.retrieveResult(pinput, poutput,
                          skinIndex, skinWeight, skinMatrix, clusterCenter)
    for v in xrange(numVertices):
        skinWeight[v] = np.maximum(0.0, skinWeight[v])
        skinWeight[v] /= np.sum(skinWeight[v])
    native.release(pinput, poutput)
    # joint creation
    dagModifier = om.MDagModifier()
    skinJoints = []
    for c in xrange(numJoints):
        newJnt = SkinJoint()
        newJnt.name = 'ssds:Joint' + str(c + 1).zfill(2)
        newJnt.bindPose = np.eye(4)
        newJnt.bindPose[3, 0:3] = clusterCenter[c]
        newJntObj = dagModifier.createNode('joint', om.MObject.kNullObj)
        dagModifier.renameNode(newJntObj, newJnt.name)
        dagModifier.doIt()
        newJointSL = om.MGlobal.getSelectionListByName(newJnt.name)
        newJnt.path = newJointSL.getDagPath(0)
        skinJoints.append(newJnt)
    bakeJointMotion(skinJoints, skinMatrix)
    # skin binding
    oma.MAnimControl.setCurrentTime(oma.MAnimControl.animationStartTime())
    dstMeshPaths, dstGroup = cloneMeshs(srcMeshPaths)
    for sj in skinJoints:
        cmds.parent(sj.name, dstGroup)
    bindToSkin(dstMeshPaths, skinIndex, skinWeight, skinJoints, numMaxInfluences)
    om.MGlobal.displayInfo('Finished')
