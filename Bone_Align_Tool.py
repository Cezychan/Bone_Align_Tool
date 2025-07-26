import maya.cmds as cmds
import maya.api.OpenMaya as om
import numpy as np 
import math

def safeGetAttr(joint, attr, default=0.0):
	if cmds.objExists(joint + "." + attr):
		return cmds.getAttr(joint + "." + attr)[0]
	return (default, default, default)

def getAxisROrder(joint):
	return safeGetAttr(joint + ".rotateOrder")

def getAxisR(joint):
	axisJoint = safeGetAttr(joint, "rotateAxis")
	axisEuler = om.MEulerRotation( 
		math.radians(axisJoint[0]), 
		math.radians(axisJoint[1]), 
		math.radians(axisJoint[2]),
		0 
	)
	return axisEuler.asMatrix()

def getJOrientR(joint):
	orientJoint = safeGetAttr(joint + ".jointOrient")[0] 
	orientEuler = om.MEulerRotation( 
		math.radians(orientJoint[0]), 
		math.radians(orientJoint[1]), 
		math.radians(orientJoint[2])
	)
	orientMatrix = orientEuler.asMatrix()
	return orientMatrix

def getGlobalR(joint):
	matrix = cmds.xform(joint, q=True, m=True, ws=True)
	return om.MTransformationMatrix(om.MMatrix(matrix)).rotation(asQuaternion=True).asMatrix()

def getParentJoint(joint):
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

def getParentR(joint):
	return getGlobalR(getParentJoint(joint))

def getWorldPosition(joint, as_vector=True):
	pos = cmds.xform(joint, query=True, worldSpace=True, translation=True)
	if as_vector:
		return om.MVector(pos[0], pos[1], pos[2])
	return pos

def getBoneVec(joint):
	parentJoint = getParentJoint(joint)
	boneVec = getJointPosition(joint) - getJointPosition(parentJoint) 
	return boneVec.normal() if boneVec.length() > 1e-6 else om.MVector(1, 0, 0) # 归一化向量

def getDeltaWorldR(v1, v2):
	if v1.length() < 1e-6 or v2.length() < 1e-6:
		return om.MMatrix()
	return v1.rotateTo(v2).asMatrix()

def alignVec(joint, targetVec, restoreLength=True):
	parentJoint = getParentJoint(joint)
	targetRotateEuler = om.MTransformationMatrix(
		getAxisR(parentJoint).inverse() * getJOrientR(parentJoint).inverse() * getParentR(parentJoint).inverse() *
		getGlobalR(parentJoint) * getDeltaWorldR(getBoneVec(joint), targetVec)
	).rotation(asQuaternion=True).asEulerRotation(
	).reorderIt(getAxisROrder(parentJoint))
	print(f"目标局部旋转欧拉角: {targetRotateEuler}")

	oriLength = getBoneVecOriLength(joint)
	print(f"原始骨骼长度: {oriLength}")

	oriChildJPos = getJointPosition(joint)

	cmds.setAttr(parentJoint + '.rotateX', math.degrees(targetRotateEuler[0]))
	cmds.setAttr(parentJoint + '.rotateY', math.degrees(targetRotateEuler[1]))
	cmds.setAttr(parentJoint + '.rotateZ', math.degrees(targetRotateEuler[2]))

	if restoreLength:
		newBoneVec = getBoneVec(joint)
		if newBoneVec.length() > 1e-6:
				scaleFactor = oriLength / newBoneVec.length()
				correctedChildPos = getJointPosition(parentJoint) + newBoneVec * scaleFactor
				cmds.xform(joint, t=(correctedChildPos.x, correctedChildPos.y, correctedChildPos.z), ws=True)
		else:
			cmds.xform(joint, t=oriChildJPos, ws=True) 

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
