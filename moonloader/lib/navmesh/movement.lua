local math_deg = math.deg
local math_atan2 = math.atan2

local function getTargetAngle(px, py, tx, ty)
    return (360 - math_deg(math_atan2(tx - px, ty - py))) % 360
end

local function getCameraAngle(px, py, tx, ty)
    return math_atan2(ty - py, tx - px) + math.pi
end

local function startRunning()
    setVirtualKeyDown(0x57, true)
    setVirtualKeyDown(0x20, true)
end

local function stopRunning()
    setVirtualKeyDown(0x57, false)
    setVirtualKeyDown(0x20, false)
end

local function runWhileDist(targetX, targetY, targetZ)
    local x, y, z = getCharCoordinates(PLAYER_PED)
    setCharHeading(PLAYER_PED, getTargetAngle(x, y, targetX, targetY))
    setCameraPositionUnfixed(0, getCameraAngle(x, y, targetX, targetY))
    repeat
        startRunning()
        wait(0)
        x, y, z = getCharCoordinates(PLAYER_PED)
    until getDistanceBetweenCoords3d(x, y, z, targetX, targetY, targetZ) <= 1

    stopRunning()
end

return {
    runWhileDist = runWhileDist,
    startRunning = startRunning,
    stopRunning = stopRunning
}
