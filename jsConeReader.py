### Pose Space Reader

def poseSpaceReader(baseParent=None, poseParent=None, color=None, side="", part="", pos=""):
    base = pmc.spaceLocator(n= side + part + pos + "psrBASE")
    goal = pmc.spaceLocator(n= side + part + pos + "psrGOAL")
    pose = pmc.spaceLocator(n= side + part + pos + "psrPOSE")
    up = pmc.spaceLocator(n= side + part + pos + "psrCONEUP")
    grp = pmc.group(empty=True, n='GRP_' + side + part + pos + 'PSR')
    
    base.overrideEnabled.set(1)
    base.overrideColor.set(9)
    
    goal.overrideEnabled.set(1)
    goal.overrideColor.set(17)
    
    pose.overrideEnabled.set(1)
    goal.overrideColor.set(16)
    
    up.visibility.set(0)

    pmc.addAttr(goal, at='double', dv=0, ln="psrAttributes", keyable=False)
    pmc.setAttr(goal.psrAttributes, e=True, channelBox=True)
    pmc.addAttr(goal, at='double', dv=90, min=0, max=180, ln="coneAngle", keyable=True)
    pmc.addAttr(goal, at='bool', dv=1, ln="coneVisibility", keyable=True)    
    pmc.addAttr(goal, at='double', dv=0, ln="resultValue", min=0, max=1, keyable=True)
    pmc.addAttr(base, at='double', dv=0, ln="resultValue", min=0, max=1,keyable=True)
    pmc.addAttr(pose, at='double', dv=0, ln="resultValue", min=0, max=1, keyable=True)
    
    cone = pmc.cone(p=(0,1,0), ax=(0,-1,0), ssw=0, esw=360, r=2, hr=1, d=3, ut=0, ch=1,
                    n=side + part + pos + 'psrCONE')[0]
    cone.overrideEnabled.set(1)
    cone.overrideShading.set(0)
    cone.overrideColor.set(color)
    
    cone.castsShadows.set(0)
    cone.receiveShadows.set(0)
    cone.motionBlur.set(0)
    cone.primaryVisibility.set(0)
    cone.smoothShading.set(0)
    cone.visibleInReflections.set(0)
    cone.visibleInRefractions.set(0)
    cone.doubleSided.set(0)
    
    for part in goal, base, pose:
        part.overrideEnabled.set(1)
        part.overrideColor.set(color)
    
    pmc.xform(goal, ws=True, t=(0,3,0))
    pmc.xform(pose, ws=True, t=(4,3,0))
    pmc.xform(up, ws=True, t=(0,0,6))
    
    pmc.connectAttr(goal.coneVisibility, cone.visibility)
    pmc.parent(cone, base)
    pmc.parent(cone, base, goal, pose, up, grp)
    pmc.aimConstraint(goal, cone, mo=True, aimVector=(0,1,0), worldUpType="object", worldUpObject = up)
    pmc.select(cl=True)
    
    if not pmc.pluginInfo('matrixNodes.mll', q=True) == 1:
        try:
            pmc.loadPlugin('matrixNodes.mll')
        except:
            print 'The plugin matrixNodes.mll could not be found.'
            print 'The plugin is required to build the nodal pose space reader. Please check your default plugins directory'
    
    base_dec = pmc.createNode('decomposeMatrix', n=side + part + pos + 'psrBASE_DECOMP')
    goal_dec = pmc.createNode('decomposeMatrix', n=side + part + pos + 'psrGOAL_DECOMP')
    pose_dec = pmc.createNode('decomposeMatrix', n=side + part + pos + 'psrPOSE_DECOMP')
    
    pmc.connectAttr(base.worldMatrix, base_dec.inputMatrix)
    pmc.connectAttr(goal.worldMatrix, goal_dec.inputMatrix)
    pmc.connectAttr(pose.worldMatrix, pose_dec.inputMatrix)
    
    goal_vec = pmc.createNode('plusMinusAverage', n=side + part + pos + 'psrGOAL_VEC')
    pose_vec = pmc.createNode('plusMinusAverage', n=side + part + pos + 'psrPOSE_VEC')
    goal_vec.operation.set(2)
    pose_vec.operation.set(2)
    
    pmc.connectAttr(goal_dec.outputTranslate, goal_vec.input3D[0])
    pmc.connectAttr(pose_dec.outputTranslate, pose_vec.input3D[0])
    pmc.connectAttr(base_dec.outputTranslate, goal_vec.input3D[1])
    pmc.connectAttr(base_dec.outputTranslate, pose_vec.input3D[1])
    
    angle = pmc.createNode('angleBetween', n=side + part + pose + 'psrVEC_ANGLE')
    pmc.connectAttr(goal_vec.output3D, angle.vector1)
    pmc.connectAttr(pose_vec.output3D, angle.vector2)
    
    angle_ratio = pmc.createNode('multiplyDivide', n= side + part + pos +'psrANGLE_RATIO')
    half_cone = pmc.createNode('multiplyDivide', n= side + part + pos +'psrHALF_CONE')
    
    pmc.connectAttr(goal.coneAngle, half_cone.input1X)
    half_cone.input2X.set(0.5)
    angle_ratio.operation.set(2)
    pmc.connectAttr(angle.angle, angle_ratio.input1X)
    pmc.connectAttr(half_cone.outputX, angle_ratio.input2X)
    
    cond = pmc.createNode('condition', n=side + part + pos + 'psrCOND')
    cond.operation.set(2)
    pmc.connectAttr(angle_ratio.outputX, cond.firstTerm)
    cond.secondTerm.set(1.0)
    cond.colorIfTrueR.set(1.0)
    pmc.connectAttr(angle_ratio.outputX, cond.colorIfFalseR)
        
    result_val = pmc.createNode('plusMinusAverage', n= side + part + pos + 'psrRESULT')
    result_val.operation.set(2)
    result_val.input1D[0].set(1.0)
    pmc.connectAttr(cond.outColorR, result_val.input1D[1])
    pmc.connectAttr(result_val.output1D, base.resultValue)
    pmc.connectAttr(result_val.output1D, goal.resultValue)
    pmc.connectAttr(result_val.output1D, pose.resultValue)
    pmc.connectAttr(result_val.output1D, pose.wireColorG)
    
    div_cone = pmc.createNode('multiplyDivide', n= side + part + pos + 'DIV_coneAngle')
    div_cone.operation.set(2)
    div_cone.input2X.set(90)
    pmc.connectAttr(goal.coneAngle, div_cone.input1X)
    pmc.connectAttr(div_cone.outputX, cone.scaleX)
    pmc.connectAttr(div_cone.outputX, cone.scaleZ)
    
    pmc.parent(up, base)
    pmc.parent(cone, base)