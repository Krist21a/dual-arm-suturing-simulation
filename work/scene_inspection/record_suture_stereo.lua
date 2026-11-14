sim = require('sim')

-- Load the project-local verification helpers into this optional add-on.
function loadSutureAudit(code)
    assert(load(code, 'suture_audit'))()
    return true
end

local outputDir = 'C:/Users/Kuo/AppData/Local/Temp/codex_suture_video/'
local leftSensor = -1
local rightSensor = -1
local frame = 0
local initialized = false

local function writeStatus(value)
    local file = io.open(outputDir .. 'status.txt', 'w')
    if file then
        file:write(value .. '\nframes=' .. tostring(frame) .. '\n')
        file:close()
    end
end

local function capture(sensor, prefix)
    sim.handleVisionSensor(sensor)
    local image, resolution = sim.getVisionSensorImg(sensor)
    local path = outputDir .. string.format('%s_%06d.png', prefix, frame)
    sim.saveImage(image, resolution, 0, path, -1)
end

function sysCall_info()
    return {autoStart = true}
end

function startSutureRecording(directory)
    if directory and #directory>0 then outputDir=directory..'/' end
    leftSensor = sim.getObject('/visionSensor_left')
    rightSensor = sim.getObject('/visionSensor_right')
    sim.setExplicitHandling(leftSensor, 1)
    sim.setExplicitHandling(rightSensor, 1)
    frame = 0
    initialized = true
    writeStatus('recording')
    return frame
end

function captureSutureFrame()
    if not initialized then startSutureRecording() end
    frame = frame + 1
    capture(leftSensor, 'left')
    capture(rightSensor, 'right')
    return frame
end

function finishSutureRecording()
    if not initialized then startSutureRecording() end
    for _ = 1, 12 do
        captureSutureFrame()
    end
    writeStatus('complete')
    return frame
end

function abortSutureRecording()
    if initialized then
        writeStatus('aborted')
    end
    return frame
end
