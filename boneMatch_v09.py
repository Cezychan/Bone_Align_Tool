import maya.cmds as cmds
import maya.api.OpenMaya as om
import numpy as np 
import math

def getRotOrder(joint):
	return cmds.getAttr(joint + ".rotateOrder")

def safeGetAttr(joint, attr, default=0.0):
	if cmds.objExists(joint + "." + attr):
		return cmds.getAttr(joint + "." + attr)[0]
	return (default, default, default)

def getRotAxis(joint):
	axisJoint = safeGetAttr(joint, "rotateAxis")
	axisEuler = om.MEulerRotation( 
		math.radians(axisJoint[0]), 
		math.radians(axisJoint[1]), 
		math.radians(axisJoint[2]),
		0
	)
	return axisEuler.asMatrix()

def getLocalRot(joint):
	localRot = cmds.getAttr(joint + ".rotate")[0]
	localRotEuler = om.MEulerRotation(
		math.radians(localRot[0]), 
		math.radians(localRot[1]), 
		math.radians(localRot[2]), 
		getRotOrder(joint)
	)
	return localRotEuler.asMatrix()

def getJointOrient(joint):
	orientJoint = cmds.getAttr(joint + ".jointOrient")[0] 
	orientEuler = om.MEulerRotation( 
		math.radians(orientJoint[0]), 
		math.radians(orientJoint[1]), 
		math.radians(orientJoint[2])
	)
	orientMatrix = orientEuler.asMatrix()
	return orientMatrix

def calInherentRot(joint):
	return getRotAxis(joint) *  getLocalRot(joint) * getJointOrient(joint)

def getGlobalRot(joint):
	matrix = cmds.xform(joint, q=True, m=True, ws=True)
	return om.MTransformationMatrix(om.MMatrix(matrix)).rotation(asQuaternion=True).asMatrix()

def calParentInf(joint):
	parents = cmds.listRelatives(joint, parent=True, fullPath=True)
	if not parents:
		return om.MMatrix()  # 单位矩阵
	return getGlobalRot(parents[0])

def getJointPosition(joint, as_vector=True):
	pos = cmds.xform(joint, query=True, worldSpace=True, translation=True)
	if as_vector:
		return om.MVector(pos[0], pos[1], pos[2])
	return pos

def matrixToEuler(matrix, rotOrder):
	euler = om.MTransformationMatrix(matrix).rotation(asQuaternion=True).asEulerRotation().reorderIt(rot_order)
	return math.degrees(euler.x), math.degrees(euler.y), math.degrees(euler.z)

def getParentJoint(childJoint):
	parents = cmds.listRelatives(childJoint, parent=True, fullPath=True)
	if not parents:
		cmds.warning(f"{childJoint} 没有父关节!")
		return (0, 0, 0)
	parentJoint = parents[0]
	if "twist" in parentJoint.lower():	# 不区分twist大小写
		grandparents = cmds.listRelatives(parentJoint, parent=True, fullPath=True)
		if grandparents:
			parentJoint = grandparents[0]
	return parentJoint

def getBoneVec(childJoint):
	parentJoint = getParentJoint(childJoint)
	boneVec = getJointPosition(childJoint) - getJointPosition(parentJoint) 
	return boneVec.normal() if boneVec.length() > 1e-6 else om.MVector(1, 0, 0) # 归一化向量

def getBoneVecOriLength(childJoint):
	parentJoint = getParentJoint(childJoint)
	boneVec = getJointPosition(childJoint) - getJointPosition(parentJoint) 
	return boneVec.length()

def calRotationBetweenVectors(v1, v2):
	if v1.length() < 1e-6 or v2.length() < 1e-6:
		return om.MMatrix()
	return v1.rotateTo(v2).asMatrix()

def alignVec(childJoint, targetVec, restoreLength=True):
	parentJoint = getParentJoint(childJoint)
	targetVecRotEuler = om.MTransformationMatrix(
		getRotAxis(parentJoint).inverse() * 
		(getGlobalRot(parentJoint) * calRotationBetweenVectors(getBoneVec(childJoint), targetVec)) *
		(getJointOrient(parentJoint) * calParentInf(parentJoint)).inverse()
	).rotation(asQuaternion=True).asEulerRotation(
	).reorderIt(getRotOrder(parentJoint))
	print(f"对齐角差: {targetVecRotEuler}")
	oriLength = getBoneVecOriLength(childJoint)
	print(f"原始骨骼长度: {oriLength}")
	oriChildPos = getJointPosition(childJoint)
	cmds.setAttr(parentJoint + '.rotateX', math.degrees(targetVecRotEuler[0]))
	cmds.setAttr(parentJoint + '.rotateY', math.degrees(targetVecRotEuler[1]))
	cmds.setAttr(parentJoint + '.rotateZ', math.degrees(targetVecRotEuler[2]))

	if restoreLength:
		newBoneVec = getBoneVec(childJoint)
		if newBoneVec.length() > 1e-6:
				scaleFactor = oriLength / newBoneVec.length()
				correctedChildPos = getJointPosition(parentJoint) + newBoneVec * scaleFactor
				cmds.xform(childJoint, t=(correctedChildPos.x, correctedChildPos.y, correctedChildPos.z), ws=True)
		else:
			cmds.xform(childJoint, t=oriChildPos, ws=True)

if __name__ == "__main__":
	selectedJoints = cmds.ls(selection=True, type='joint')
	if not selectedJoints:
		cmds.warning("至少选择一个JOINT!")
	elif len(selectedJoints) == 1:
		targetVec = np.array([0, -1, 0])
		v2 = om.MVector(targetVec).normal()
		alignVec(selectedJoints[0], v2)
	elif len(selectedJoints) >= 2:
		targetJoint = selectedJoints[-1]
		targetVec = getBoneVec(targetJoint)
		print("="*70)
		print(f"使用关节 '{targetJoint}' 的方向作为目标向量:")
		print(f"目标向量: ({targetVec.x:.4f}, {targetVec.y:.4f}, {targetVec.z:.4f})")
		print(f"将对齐 {len(selectedJoints)-1} 个骨骼到该方向")
		for childJoint in selectedJoints[:-1]:
			print("-"*70)
			print(f"已对齐关节: {childJoint} -> 目标方向: {targetVec}")
			alignVec(childJoint, targetVec)
		print("="*70)

