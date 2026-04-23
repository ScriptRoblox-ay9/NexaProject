local movement = require("navmesh.movement")
local los_cache = require("navmesh.los_cache")

local function cmd_path(nav_mesh, x, y, z)
    local tx, ty, tz = tonumber(x), tonumber(y), tonumber(z)
    if not tx or not ty or not tz then return false end
    local px, py, pz = getCharCoordinates(PLAYER_PED)
    local path = nav_mesh:generate_path_hybrid(px, py, pz, tx, ty, tz)
    if not path then return false end
    lua_thread.create(function()
        for i = 1, #path do
            movement.runWhileDist(path[i][1], path[i][2], path[i][3])
        end
    end)
    return true
end

local function cmd_pathdebug(nav_mesh, x1, y1, z1, x2, y2, z2)
    local ax, ay, az = tonumber(x1), tonumber(y1), tonumber(z1)
    local bx, by, bz = tonumber(x2), tonumber(y2), tonumber(z2)
    if not ax or not ay or not az or not bx or not by or not bz then return false end
    local p1 = nav_mesh:get_point_id_from_coords(ax, ay, az)
    local p2 = nav_mesh:get_point_id_from_coords(bx, by, bz)
    if not p1 or not p2 then
        sampAddChatMessage("NavMesh: point not found for given coords", 0xFF4444)
        return false
    end

    local pos1 = nav_mesh.last_id_to_pos[p1]
    local pos2 = nav_mesh.last_id_to_pos[p2]
    if not pos1 or not pos2 then
        sampAddChatMessage("NavMesh: point data missing", 0xFF4444)
        return false
    end

    local ok, reason = nav_mesh:can_connect_points(pos1[1], pos1[2], pos1[3], pos2[1], pos2[2], pos2[3], true)
    if ok then
        sampAddChatMessage("NavMesh: connection OK", 0x00FF00)
    else
        sampAddChatMessage("NavMesh: blocked -> " .. tostring(reason), 0xFF4444)
    end
    return ok
end

local function cmd_pathpos()
    local x, y, z = getCharCoordinates(PLAYER_PED)
    sampAddChatMessage(string.format("pos: %.3f %.3f %.3f", x, y, z), 0xFFFFFF)
    return true
end

local function cmd_pathstats(nav_mesh)
    local stats = nav_mesh:get_stats()
    local scan_calls = stats.scan_calls
    local update_calls = stats.update_calls
    local avg_scan_ms = scan_calls > 0 and (stats.scan_time / scan_calls) * 1000 or 0
    local avg_update_ms = update_calls > 0 and (stats.update_time / update_calls) * 1000 or 0
    sampAddChatMessage(string.format("NavMesh stats: scans=%d avg_scan=%.3fms updates=%d avg_update=%.3fms", scan_calls, avg_scan_ms, update_calls, avg_update_ms), 0x00FF00)
    sampAddChatMessage(string.format("NavMesh cache: links=%d hits=%d misses=%d", stats.link_cache_size, stats.link_cache_hits, stats.link_cache_misses), 0x00FF00)
    return true
end

local function register(nav)
    sampRegisterChatCommand("path", function(args)
        local x, y, z = args:match("([%-%d%.]+) ([%-%d%.]+) ([%-%d%.]+)")
        cmd_path(nav, x, y, z)
    end)

    sampRegisterChatCommand("pathdebug", function(args)
        local x1, y1, z1, x2, y2, z2 = args:match("([%-%d%.]+) ([%-%d%.]+) ([%-%d%.]+) ([%-%d%.]+) ([%-%d%.]+) ([%-%d%.]+)")
        cmd_pathdebug(nav, x1, y1, z1, x2, y2, z2)
    end)

    sampRegisterChatCommand("pathpos", function() cmd_pathpos() end)
    sampRegisterChatCommand("pathstats", function() cmd_pathstats(nav) end)
    sampRegisterChatCommand("pathclear", function()
        los_cache.clear()
        nav.mesh = {}
        nav.last_points_local = {}
        sampAddChatMessage("NavMesh cache cleared", 0x00FF00)
    end)
end

return {
    register = register
}
