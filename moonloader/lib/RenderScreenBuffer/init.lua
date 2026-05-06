---@diagnostic disable: undefined-global

local ffi = require("ffi")
ffi.cdef [[

  struct RECT {
    long    left;
    long    top;
    long    right;
    long    bottom;
  };

  void* RenderScreenBufferInitialize(unsigned int device, struct RECT rect);
  void RenderScreenBufferShutdown(void* core);
  void RenderScreenBufferInvalidate(void* core);
  void* RenderScreenBufferGetBuffer(void* core);
]]

local libPath = getWorkingDirectory() .. [[\lib\RenderScreenBuffer\RenderScreenBuffer]]
local lib = ffi.load(libPath)

local RenderScreenBuffer = {}
RenderScreenBuffer.__index = RenderScreenBuffer

function RenderScreenBuffer:new(minX, minY, maxX, maxY)
  local instance = setmetatable({}, RenderScreenBuffer)

  if minX == nil or minY == nil or maxX == nil or maxY == nil then
    minX, minY = 0, 0
    maxX, maxY = getScreenResolution()
  end

  local rect = ffi.new("struct RECT")
  rect.left = minX
  rect.top = minY
  rect.right = maxX
  rect.bottom = maxY
  instance.core = lib.RenderScreenBufferInitialize(getD3DDevicePtr(), rect)

  addEventHandler("onD3DDeviceLost", function()
    lib.RenderScreenBufferInvalidate(instance.core)
  end)

  addEventHandler("onScriptTerminate", function(scr, quitGame)
    if scr == thisScript() then
      lib.RenderScreenBufferShutdown(instance.core)
    end
  end)

  return instance
end

function RenderScreenBuffer:get()
  return lib.RenderScreenBufferGetBuffer(self.core)
end

return RenderScreenBuffer
