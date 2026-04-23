local math_floor = math.floor
local cfg = require("navmesh.config")

local line_of_sight_cache = {}
local cache_size = 0

local function clear()
    line_of_sight_cache = {}
    cache_size = 0
end

local function los_cache_key(x1, y1, z1, x2, y2, z2, checkBuildings, checkVehicles, checkPeds, checkObjects, checkDummies, seeThroughStuff, ignoreSomeObjectsForCamera, shootThrough)
    local qx1 = math_floor(x1 * 10 + 0.5)
    local qy1 = math_floor(y1 * 10 + 0.5)
    local qz1 = math_floor(z1 * 10 + 0.5)
    local qx2 = math_floor(x2 * 10 + 0.5)
    local qy2 = math_floor(y2 * 10 + 0.5)
    local qz2 = math_floor(z2 * 10 + 0.5)
    return qx1 .. "," .. qy1 .. "," .. qz1 .. "," .. qx2 .. "," .. qy2 .. "," .. qz2 .. "," .. (checkBuildings and 1 or 0) .. (checkVehicles and 1 or 0) .. (checkPeds and 1 or 0) .. (checkObjects and 1 or 0) .. (checkDummies and 1 or 0) .. (seeThroughStuff and 1 or 0) .. (ignoreSomeObjectsForCamera and 1 or 0) .. (shootThrough and 1 or 0)
end

local function process(x1, y1, z1, x2, y2, z2, checkBuildings, checkVehicles, checkPeds, checkObjects, checkDummies, seeThroughStuff, ignoreSomeObjectsForCamera, shootThrough)
    local key = los_cache_key(x1, y1, z1, x2, y2, z2, checkBuildings, checkVehicles, checkPeds, checkObjects, checkDummies, seeThroughStuff, ignoreSomeObjectsForCamera, shootThrough)
    local cached = line_of_sight_cache[key]
    if cached then return cached[1], cached[2] end
    if cache_size > cfg.cache_cleanup_threshold or cache_size > cfg.cache_max_size then clear() end
    local result, col_point = processLineOfSight(x1, y1, z1, x2, y2, z2, checkBuildings, checkVehicles, checkPeds, checkObjects, checkDummies, seeThroughStuff, ignoreSomeObjectsForCamera, shootThrough)
    line_of_sight_cache[key] = {result, col_point}
    cache_size = cache_size + 1
    return result, col_point
end

return {
    clear = clear,
    process = process
}
