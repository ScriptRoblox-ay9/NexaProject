local NavMesh = require("navmesh.navmesh")
local render = require("navmesh.render")
local commands = require("navmesh.commands")

function main()
    local nav = NavMesh.new()
    nav:init()
    commands.register(nav)

    while true do
        wait(0)
        nav:update_mesh()
        render.renderNavMesh(nav)
    end
end