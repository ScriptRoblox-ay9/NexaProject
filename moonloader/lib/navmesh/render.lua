local math_sqrt = math.sqrt

local function is_point_in_front_of_camera(x, y, z)
    local cx, cy, cz = getActiveCameraCoordinates()
    local lx, ly, lz = getActiveCameraPointAt()
    local dx, dy, dz = lx - cx, ly - cy, lz - cz
    local len = math_sqrt(dx * dx + dy * dy + dz * dz)
    if len < 0.001 then return false end
    local inv_len = 1 / len
    local px, py, pz = x - cx, y - cy, z - cz
    return px * dx * inv_len + py * dy * inv_len + pz * dz * inv_len > 0
end

local function renderLine3D(x1, y1, z1, x2, y2, z2, w, c)
    if is_point_in_front_of_camera(x1, y1, z1) and is_point_in_front_of_camera(x2, y2, z2) then
        local sx1, sy1 = convert3DCoordsToScreen(x1, y1, z1)
        local sx2, sy2 = convert3DCoordsToScreen(x2, y2, z2)
        if sx1 and sy1 and sx2 and sy2 then
            local sw, sh = getScreenResolution()
            if sx1 >= 0 and sx1 <= sw and sy1 >= 0 and sy1 <= sh and sx2 >= 0 and sx2 <= sw and sy2 >= 0 and sy2 <= sh then renderDrawLine(sx1, sy1, sx2, sy2, w, c) end
        end
    end
end

local LAYER_COLORS = {0x60FFFFFF, 0x5080FFFF, 0x50FF80FF, 0x50FFFF80, 0x5080FF80}
local LAYER_COLOR_COUNT = #LAYER_COLORS
local RENDER_DIST_SQ = 6400.0
local LINE_WIDTH = 0.8

local function should_render_node(node_data, x1, y1, px, py, pz, node_points_idx, node_timestamp_idx)
    if node_data[node_timestamp_idx] == 0 then return false end
    local points = node_data[node_points_idx]
    if #points == 0 then return false end
    local dx, dy, dz = x1 - px, y1 - py, points[1] - pz
    return dx * dx + dy * dy + dz * dz <= RENDER_DIST_SQ
end

local function render_connection(x1, y1, x2, y2, points, neighbor_points, conn)
    local z1, z2 = points[conn[1]], neighbor_points[conn[2]]
    if z1 and z2 then
        local color = LAYER_COLORS[((conn[1] - 1) % LAYER_COLOR_COUNT) + 1]
        renderLine3D(x1, y1, z1, x2, y2, z2, LINE_WIDTH, color)
    end
end

local function renderNavMesh(nav_mesh)
    local px, py, pz = getCharCoordinates(PLAYER_PED)
    local mesh = nav_mesh.mesh
    for idx, node_data in pairs(mesh) do
        local x1, y1 = nav_mesh:get_coords_from_node(idx)
        if should_render_node(node_data, x1, y1, px, py, pz, nav_mesh.NODE_POINTS, nav_mesh.NODE_TIMESTAMP) then
            local points = node_data[nav_mesh.NODE_POINTS]
            for neighbor_idx, connections in pairs(node_data[nav_mesh.NODE_NEIGHBORS]) do
                local neighbor_node = mesh[neighbor_idx]
                if neighbor_node and neighbor_node[nav_mesh.NODE_TIMESTAMP] ~= 0 then
                    local x2, y2 = nav_mesh:get_coords_from_node(neighbor_idx)
                    local neighbor_points = neighbor_node[nav_mesh.NODE_POINTS]
                    for i = 1, #connections do
                        render_connection(x1, y1, x2, y2, points, neighbor_points, connections[i])
                    end
                end
            end
        end
    end
end

return {
    renderNavMesh = renderNavMesh
}
