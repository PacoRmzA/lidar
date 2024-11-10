sim = require('sim')

function sysCall_init() 
    visionSensor1Handle = sim.getObject("../sensor1")
    visionSensor2Handle = sim.getObject("../sensor2")
    joint1Handle = sim.getObject("../joint1")
    joint2Handle = sim.getObject("../joint2")
    sensorRefHandle = sim.getObject("../ref")

    maxScanDistance = 4
    sim.setObjectFloatParam(visionSensor1Handle, sim.visionfloatparam_far_clipping, maxScanDistance)
    sim.setObjectFloatParam(visionSensor2Handle, sim.visionfloatparam_far_clipping, maxScanDistance)
    maxScanDistance_ = maxScanDistance * 0.9999

    scanningAngle = 270 * math.pi / 180
    sim.setObjectFloatParam(visionSensor1Handle, sim.visionfloatparam_perspective_angle, scanningAngle / 2)
    sim.setObjectFloatParam(visionSensor2Handle, sim.visionfloatparam_perspective_angle, scanningAngle / 2)

    sim.setJointPosition(joint1Handle, -scanningAngle / 4)
    sim.setJointPosition(joint2Handle, scanningAngle / 4)
    red = {1, 0, 0}
    lines = sim.addDrawingObject(sim.drawing_lines, 1, 0, -1, 1000, red)
    showLines = true
end

function sysCall_cleanup() 
    sim.removeDrawingObject(lines)
end 

function sysCall_sensing() 
    measuredData = {}
    measuredData2 = {}
    if notFirstHere then
        sim.addDrawingObjectItem(lines, nil)
        r, t1, u1 = sim.readVisionSensor(visionSensor1Handle)
        r, t2, u2 = sim.readVisionSensor(visionSensor2Handle)
    
        m1 = sim.getObjectMatrix(visionSensor1Handle)
        m01 = sim.getObjectMatrix(sensorRefHandle)
        m01 = sim.getMatrixInverse(m01)
        m01 = sim.multiplyMatrices(m01, m1)
        m2 = sim.getObjectMatrix(visionSensor2Handle)
        m02 = sim.getObjectMatrix(sensorRefHandle)
        m02 = sim.getMatrixInverse(m02)
        m02 = sim.multiplyMatrices(m02, m2)

        -- Process sensor 1 data
        if u1 then
            p = {0, 0, 0}
            p = sim.multiplyVector(m1, p)
            t = {p[1], p[2], p[3], 0, 0, 0}
            for j = 0, u1[2] - 1, 1 do
                for i = 0, u1[1] - 1, 1 do
                    w = 2 + 4 * (j * u1[1] + i)
                    v1 = u1[w + 1]
                    v2 = u1[w + 2]
                    v3 = u1[w + 3]
                    v4 = u1[w + 4]
                    if (v4 < maxScanDistance_) then
                        p = {v1, v2, v3}
                        p = sim.multiplyVector(m01, p)
                        table.insert(measuredData, p[1])
                        table.insert(measuredData, p[2])
                        table.insert(measuredData, p[3])

                        -- Calculate distance and angle
                        local distance = math.sqrt(p[1]^2 + p[2]^2 + p[3]^2)
                        local angle = math.atan2(p[2], p[1]) -- angle in radians
                        
                        table.insert(measuredData2, distance)
                        table.insert(measuredData2, angle)
                        
                        -- print(string.format("Object detected: %.2f meters away at coordinates [x: %.2f, y: %.2f, z: %.2f], angle: %.2f radians", 
                        --                    distance, p[1], p[2], p[3], angle))
                    end
                    if showLines then
                        p = {v1, v2, v3}
                        p = sim.multiplyVector(m1, p)
                        t[4] = p[1]
                        t[5] = p[2]
                        t[6] = p[3]
                        sim.addDrawingObjectItem(lines, t)
                    end
                end
            end
        end

        -- Process sensor 2 data
        if u2 then
            p = {0, 0, 0}
            p = sim.multiplyVector(m2, p)
            t = {p[1], p[2], p[3], 0, 0, 0}
            for j = 0, u2[2] - 1, 1 do
                for i = 0, u2[1] - 1, 1 do
                    w = 2 + 4 * (j * u2[1] + i)
                    v1 = u2[w + 1]
                    v2 = u2[w + 2]
                    v3 = u2[w + 3]
                    v4 = u2[w + 4]
                    if (v4 < maxScanDistance_) then
                        p = {v1, v2, v3}
                        p = sim.multiplyVector(m02, p)
                        table.insert(measuredData, p[1])
                        table.insert(measuredData, p[2])
                        table.insert(measuredData, p[3])

                        -- Calculate distance and angle
                        local distance = math.sqrt(p[1]^2 + p[2]^2 + p[3]^2)
                        local angle = math.atan2(p[2], p[1]) -- angle in radians
                        
                        table.insert(measuredData2, distance)
                        table.insert(measuredData2, angle)
                        
                        --print(string.format("Object detected: %.2f meters away at coordinates [x: %.2f, y: %.2f, z: %.2f], angle: %.2f radians", 
                        --                    distance, p[1], p[2], p[3], angle))
                    end
                    if showLines then
                        p = {v1, v2, v3}
                        p = sim.multiplyVector(m2, p)
                        t[4] = p[1]
                        t[5] = p[2]
                        t[6] = p[3]
                        sim.addDrawingObjectItem(lines, t)
                    end
                end
            end
        end
    end
    if #measuredData2 > 0 then
        sim.writeCustomTableData(sim.handle_scene, "lidarData", measuredData2)
        --print("Data stored in the scene object:", measuredData2)
    else
        print("No measured data to store")
    end
    notFirstHere = true
end
