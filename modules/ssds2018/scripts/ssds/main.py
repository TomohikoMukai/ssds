# -*- coding: utf-8 -*-
from maya import cmds
from maya import mel
from maya.api import OpenMaya as om
from maya.api import OpenMayaAnim as oma
import math
import numpy as np
import native

candidates = None

class SkinJoint:
    def __init__(self, path = None, name = '', bindPose = np.eye(4)):
        self.path = path
        if path:
            self.name = om.MFnTransform(path).name()
            self.bindPose = np.eye(4)
            t = om.MFnTransform(path).translation(om.MSpace.kTransform)
            self.bindPose[3, 0:3] = np.array([t.x, t.y, t.z])
        else:
            self.name = name
            self.bindPose = bindPose.copy()

    def __str__(self):
        return self.name


def getMeshs():
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


def getJoints():
    asl = om.MGlobal.getActiveSelectionList()
    itsl = om.MItSelectionList(asl)
    jointPaths = []
    while not itsl.isDone():
        try:
            dagPath = itsl.getDagPath()
        except:  continue
        finally: itsl.next()
        if not dagPath:
            continue
        apiType = dagPath.apiType()
        if apiType == om.MFn.kJoint:
            jointPaths.append(dagPath)
    return jointPaths


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
        retval[i, 0:len(neighbor[i])] = sorted(neighbor[i])
    return retval

 
def findSkinCluster(path):
    if path.hasFn(om.MFn.kMesh):
        mesh = om.MFnMesh(path)
        inMeshPlug = mesh.findPlug('inMesh', True)
        srcs = inMeshPlug.connectedTo(True, False)
        for s in srcs:
            nobj = s.node()
            if nobj.hasFn(om.MFn.kSkinClusterFilter):
                return nobj
    elif path.hasFn(om.MFn.kTransform):
        jfn = om.MFnTransform(path)
        plug = jfn.findPlug('worldMatrix', True)
        plug = plug.elementByLogicalIndex(0)
        dsts = plug.connectedTo(False, True)
        for d in dsts:
            nobj = d.node()
            if nobj.hasFn(om.MFn.kSkinClusterFilter):
                return nobj
    return None


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
        meshName = om.MFnDagNode(mesh.parent(0)).name()
        skinNode = findSkinCluster(meshPath)
        if skinNode:
            cmds.skinCluster(meshName, edit = True, unbind = True)
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
    cloneGroup = cmds.group(empty = True, world = True, name = 'ssds:result')
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
        cmds.setKeyframe(at = ['tx', 'ty', 'tz', 'rx', 'ry', 'rz', 'sx', 'sy', 'sz'])
        frame = frame + 1
        ctime = ctime + 1
    oma.MAnimControl.setCurrentTime(startTime)


def build(minNumJoints = 1,
          maxNumJoints = 1,
          transformTypes = [2],
          numMaxInfluences = 4,
          numIterations = 5,
          smoothness = 0,
          numRings = 0,
          uniformSample = True):
    srcMeshPaths = getMeshs()
    if len(srcMeshPaths) == 0:
        raise Exception('Select mesh')
    srcMeshNames = []
    for p in srcMeshPaths:
        srcMeshNames.append(om.MFnMesh(p).name())
    om.MGlobal.displayInfo('SSDS v.2020.9.6')
    om.MGlobal.displayInfo(' # joints: '     + str(minNumJoints) + ', ' + str(maxNumJoints))
    om.MGlobal.displayInfo(' # iterations: ' + str(numIterations))
    om.MGlobal.displayInfo(' transform: '    + str(transformTypes))
    om.MGlobal.displayInfo(' smoothness: '   + str(smoothness))
    om.MGlobal.displayInfo(' neighbors: '    + str(numRings))
    if not cmds.namespace(exists = 'ssds'):
        cmds.namespace(add = 'ssds')

    global candidates
    candidates = []
    candidateStats = []
    njset = sorted(set([minNumJoints, (minNumJoints + maxNumJoints) / 2, maxNumJoints]))
        
    mainbar = mel.eval('$tmp = $gMainProgressBar')
    pbar = cmds.progressBar(mainbar,
        edit = True,
        beginProgress = True,
        progress = 0,
        status = 'Initializing',
        minValue = 0,
        maxValue = 2 + numIterations * len(njset) * len(transformTypes),
        isInterruptable = False)    

    # data acquisition
    oma.MAnimControl.setCurrentTime(oma.MAnimControl.animationStartTime())
    shapeSample = sampleShapes(srcMeshPaths)
    initPos = shapeSample[0]
    numVertices = shapeSample.shape[1]
    neighborVertices = concatenateNeighborLists(srcMeshPaths)
    cmds.progressBar(pbar, edit = True, step = 1, status = 'Acquired source data')

    for njnts in njset:
        for ttype in transformTypes:
            # initialize native module
            pinput, poutput = native.initialize(
                initPos, shapeSample, neighborVertices, njnts, numMaxInfluences, numRings)
            # skinning decomposition
            if uniformSample:
                njnts = native.clusterVerticesPcenter(pinput, poutput, ttype)
            else:
                njnts = native.clusterVerticesAdaptive(pinput, poutput, ttype)
            for it in xrange(numIterations):
                native.updateSkinWeight(pinput, poutput, smoothness)
                native.updateJointTransform(pinput, poutput, ttype)
                cmds.progressBar(pbar, edit = True, step = 1, status = 'Iteration - ' + str(it + 1) + '/' + str(numIterations))
            # data retrieval from native module
            si = -np.ones([numVertices, numMaxInfluences], dtype = np.int)
            sw = np.zeros([numVertices, numMaxInfluences])
            sm = np.zeros([njnts, shapeSample.shape[0], 4, 4])
            clusterCenter = np.zeros([njnts, 3])
            native.retrieveResult(pinput, poutput,
                                  si, sw, sm, clusterCenter)
            for v in xrange(numVertices):
                sw[v] = np.maximum(0.0, sw[v])
                sw[v] /= np.sum(sw[v])
            native.release(pinput, poutput)
            candidateStats.append(
                'Joints: ' + str(njnts).zfill(2) + 
                ', Type: ' + ['  T', ' TR', 'TRS'][ttype])
            candidates.append({
                'joints'    : njnts,
                'transform' : ttype,
                'influence' : numMaxInfluences,
                'index'     : np.copy(si),
                'matrix'    : np.copy(sm),
                'weight'    : np.copy(sw),
                'message'   : candidateStats[-1]})

    # joint creation
    dagModifier = om.MDagModifier()
    skinJoints = []
    for c in xrange(maxNumJoints):
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
    bakeJointMotion(skinJoints, sm)
    cmds.progressBar(pbar, edit = True, step = 1, status = 'Finalized')

    # skin binding
    oma.MAnimControl.setCurrentTime(oma.MAnimControl.animationStartTime())
    dstMeshPaths, dstGroup = cloneMeshs(srcMeshPaths)
    for sj in skinJoints:
        cmds.parent(sj.name, dstGroup)
    bindToSkin(dstMeshPaths, si, sw, skinJoints, numMaxInfluences)
    cmds.progressBar(pbar, edit = True, endProgress = True)
    
    return candidateStats


def select(cid):
    try:
        rootPath = om.MGlobal.getSelectionListByName('ssds:result').getDagPath(0)
    except:
        om.MGlobal.displayError('ssds model was not found')
        return False
    global candidates
    om.MGlobal.displayInfo('Switch to ' + candidates[cid]['message'])
    numJoints  = candidates[cid]['joints']
    influence  = candidates[cid]['influence']
    skinIndex  = candidates[cid]['index']
    skinWeight = candidates[cid]['weight']
    skinMatrix = candidates[cid]['matrix']

    animStartTime = oma.MAnimControl.animationStartTime()
    oma.MAnimControl.setCurrentTime(animStartTime)
    rootTrans = om.MFnTransform(rootPath).translation(om.MSpace.kWorld)
    om.MFnTransform(rootPath).setTranslation(om.MVector.kZeroVector, om.MSpace.kWorld)

    children = cmds.listRelatives('ssds:result', children = True)
    cmds.select(children)
    meshs = getMeshs()
    joints = getJoints()
    skinJoints = [SkinJoint(j) for j in joints[0:numJoints]]
    for sj in joints[:numJoints]:
        cmds.setAttr(om.MFnTransform(sj).name() + '.visibility', True)
    for hj in joints[numJoints:]:
        cmds.setAttr(om.MFnTransform(hj).name() + '.visibility', False)
    bakeJointMotion(skinJoints, skinMatrix)
    bindToSkin(meshs, skinIndex, skinWeight, skinJoints, influence)

    om.MFnTransform(rootPath).setTranslation(rootTrans, om.MSpace.kWorld)