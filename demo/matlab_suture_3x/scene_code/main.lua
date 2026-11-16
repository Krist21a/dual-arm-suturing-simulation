require('defaultMainScript')
local original=sysCall_joint
function sysCall_joint(d)
    if d.mode==sim.jointmode_kinematic then return {pos=d.targetPos or d.pos,vel=0,accel=0,immobile=false}end
    return original(d)
end
