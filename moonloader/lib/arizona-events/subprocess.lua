local BitStream = require("arizona-events.bitstream")

local subprocess = {}
local thread


local pending = {}
local pending_count = 0

local function flush_pending()
  if pending_count == 0 then return end
  local ffi = require("ffi")
  local clock
  if jit.os == "Windows" then
    ffi.cdef "unsigned __stdcall GetTickCount(void);"
    clock = ffi.C.GetTickCount
  else
    clock = function() return math.floor(os.clock() * 1000) end
  end
  local now = clock()
  local i = 1
  while i <= pending_count do
    local task = pending[i]
    
    if task then
      local bs = BitStream.new()
      task.write(bs, task.packet)
      BitStream.emul(task.id, bs)
      BitStream.delete(bs)
      pending[i] = nil
    end
    i = i + 1
  end
  
  local new_pending = {}
  local new_count = 0
  for j = 1, pending_count do
    if pending[j] ~= nil then
      new_count = new_count + 1
      new_pending[new_count] = pending[j]
    end
  end
  pending = new_pending
  pending_count = new_count
end

function subprocess.push(id, packet, write)
  pending_count = pending_count + 1
  pending[pending_count] = { id = id, packet = packet, write = write }
  if thread and thread:status() ~= "yielded" then
    thread:run()
  end
end

if lua_thread then
  thread = lua_thread.create_suspended(function()
    while true do
      if pending_count > 0 then
        flush_pending()
        wait(0)
      else
        
        wait(50)
      end
    end
  end)
else
  thread = {}
  function thread:run() end
  function thread:status() end
end

return subprocess
