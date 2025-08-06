--[[
   simulate a silvus radio providing time of flight range
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

PARAM_TABLE_KEY = 53
PARAM_TABLE_PREFIX = "SSIM_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 63), 'could not add param table')

--[[
  // @Param: SSIM_ENABLE
  // @DisplayName: enable Silvus simulator
  // @Description: Enable Silvus simulator
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local SSIM_ENABLE = bind_add_param('ENABLE',  1, 0)
if SSIM_ENABLE:get() == 0 then
   return
end

--[[
  // @Param: SSIM_DIST_OFS
  // @DisplayName: Silvus distance offset
  // @Description: Silvus distance offset
  // @Units: m
  // @User: Standard
--]]
local SSIM_DIST_OFS = bind_add_param('DIST_OFS', 2, 0)

--[[
  // @Param: SSIM_DIST_MUL
  // @DisplayName: Silvus distance multiplier
  // @Description: Silvus distance multiplier
  // @Units: m
  // @User: Standard
--]]
local SSIM_DIST_MUL = bind_add_param('DIST_MUL', 3, 30)

--[[
  // @Param: SSIM_LISTEN_PORT
  // @DisplayName: Silvus listen port
  // @Description: Silvus listen port
  // @User: Standard
--]]
local SSIM_LISTEN_PORT = bind_add_param('LISTEN_PORT', 4, 8003)

--[[
  // @Param: SSIM_AGE_MAX_MS
  // @DisplayName: Silvus age max MS
  // @Description: Silvus age max MS
  // @User: Standard
--]]
local SSIM_AGE_MAX_MS = bind_add_param('AGE_MAX_MS', 5, 5000)


--[[
  // @Param: SSIM_NUM_RADIOS
  // @DisplayName: Silvus number of ground radios
  // @Description: Silvus number of ground radios
  // @Range: 1 8
  // @User: Standard
--]]
local SSIM_NUM_RADIOS = bind_add_param('NUM_RADIOS', 13, 0)

--[[
  // @Param: SSIM_GND1_NODEID
  // @DisplayName: Silvus node ID for first ground radio
  // @Description: Silvus node ID for first ground radio
  // @User: Standard
--]]

--[[
  // @Param: SSIM_GND1_LAT
  // @DisplayName: Silvus ground radio 1 latitude
  // @Description: Silvus ground radio 1 latitude
  // @Units: deg
  // @User: Standard
--]]

--[[
  // @Param: SSIM_GND1_LON
  // @DisplayName: Silvus ground radio 1 longitude
  // @Description: Silvus ground radio 1 longitude
  // @Units: deg
  // @User: Standard
--]]

--[[
  // @Param: SSIM_GND1_ALT
  // @DisplayName: Silvus ground radio 1 height AMSL
  // @Description: Silvus ground radio 1 height AMSL
  // @Units: m
  // @User: Standard
--]]

local MAX_GROUND_RADIOS = 8

local SSIM_GND_NODEID = {}
local SSIM_GND_LAT = {}
local SSIM_GND_LON = {}
local SSIM_GND_ALT = {}

--add the local node id 
SSIM_GND_NODEID[SSIM_NUM_RADIOS:get()+1] = bind_add_param('LCL_NODEID', 14, 0)

-- clamp number of radios
if SSIM_NUM_RADIOS:get() > MAX_GROUND_RADIOS then
   SSIM_NUM_RADIOS:set(MAX_GROUND_RADIOS)
end

--[[
   create the parameters per ground radio (beacon)
--]]
for r = 1, SSIM_NUM_RADIOS:get() do
   SSIM_GND_NODEID[r] = bind_add_param(string.format('GND%u_NODEID',r), 20+(r-1)*5, 0)
   SSIM_GND_LAT[r] = bind_add_param(string.format('GND%u_LAT',r),       21+(r-1)*5, 0)
   SSIM_GND_LON[r] = bind_add_param(string.format('GND%u_LON',r),       22+(r-1)*5, 0)
   SSIM_GND_ALT[r] = bind_add_param(string.format('GND%u_ALT',r),       23+(r-1)*5, 0)
end

local listen_sock = nil      -- used to listen for connects
local connection_sock = nil  -- connection from http client
local http_request = nil     -- data being read from client
local request_start = nil    -- time connection was created
local REQUEST_TIMEOUT = 250  -- timeout for client requests

local json = require("json")
local json_log = nil

gcs:send_text(MAV_SEVERITY.INFO, string.format("SilvusSim: starting with %u beacons", #SSIM_GND_ALT))

local function send_json(json, chunked)
   local lenchu = "Transfer-Encoding: chunked"
   local chu = "\n0\n"
   if not chunked then
      lenchu = "Content-Length: "..#json
      chu = ""
   end
   local msg = string.format([[HTTP/1.0 200 OK
Content-Type: application/json
Cache-Control: no-cache
%s
Server: silvus sim

%s%s]], lenchu, json, chu)
   msg = string.gsub(msg, "\n", "\r\n")
   connection_sock:send(msg, #msg)
end

local function send_noise_level()
   -- gcs:send_text(MAV_SEVERITY.INFO, string.format("SilvusSim: send_noise_level"))
   noise_level = 17 + math.random(-5,5)
   local json = string.format([[{"result" : [%s],"id" : "sbkb5u0c", "jsonrpc" : "2.0"}]], noise_level)
   send_json(json, false)
end

local function send_nbr_rssi()
   r1 = 2+math.random(-1,1)
   r2 = 4+math.random(-1,1)
   r3 = 6+math.random(-1,1)
   r4 = 8+math.random(-1,1)
   local json = string.format([[{"result" : [%s,%s,%s,%s],"id" : "sbkb5u0c", "jsonrpc" : "2.0"}]], r1,r2,r3,r4)
   send_json(json, false)
end

local function send_link_throughput()
   link_throughput = 5 + math.random(-5,5)
   local json = string.format([[{"result" : [%s],"id" : "sbkb5u0c", "jsonrpc" : "2.0"}]], link_throughput)
   send_json(json, false)
end

local function send_network_status()
   -- currently just sending one starting with each node id and ending with a random one
   local network_str = ""
   for i = 1, #SSIM_GND_NODEID do
      local snr = math.random(0,10)
      local othernode = math.random(1, #SSIM_GND_NODEID)
      if #network_str > 0 then
         network_str = network_str .. ","
      end
      network_str = network_str .. string.format([["%u","%u","%u"]], SSIM_GND_NODEID[i]:get(), SSIM_GND_NODEID[othernode]:get(), snr)
   end
   local json = string.format([[{"result" : [%s],"id" : "sbkb5u0c", "jsonrpc" : "2.0"}]], network_str)
   send_json(json, true)
end

local function send_mcs()
   mcs = math.random(5,15)
   local json = string.format([[{"result" : [%s],"id" : "sbkb5u0c", "jsonrpc" : "2.0"}]], mcs)
   send_json(json, false)
end

local function send_weakest_link()
   n1 = SSIM_GND_NODEID[math.random(1, #SSIM_GND_NODEID)]:get()
   n2 = SSIM_GND_NODEID[1]:get()
   snr = math.random(44,46)
   reuse = math.random(0,3)
   local json = string.format([[{"result" : [%s,%s,%s,%s],"id" : "sbkb5u0c", "jsonrpc" : "2.0"}]], n1,n2,snr,reuse)
   send_json(json, false)
end

--[[
   get ground radio location
--]]
local function get_radio_location(radio_idx)
   local loc = Location()
   if not loc then
      return nil
   end
   if radio_idx < 1 or radio_idx > #SSIM_GND_LAT then
      return nil
   end
   local lat = SSIM_GND_LAT[radio_idx]:get()
   local lon = SSIM_GND_LON[radio_idx]:get()
   local alt = SSIM_GND_ALT[radio_idx]:get()
   if lat == 0 or lon == 0 then
      return nil
   end
   loc:lat(math.floor(lat*1.0e7))
   loc:lng(math.floor(lon*1.0e7))
   loc:alt(math.floor(alt*100))
   return loc
end

--[[
   get range in TOF units (100ns), given offset of beacon from home
--]]
local function get_range_ticks(radio_idx)
   return 17+math.random(-2,2)
   -- local gps_loc = gps:location(0)
   -- if not gps_loc then
   --    return nil
   -- end
   -- local loc = get_radio_location(radio_idx)
   -- if not loc then
   --    return nil
   -- end
   -- local range_2D_m = gps_loc:get_distance(loc)
   -- if range_2D_m > 300000 then
   --    return nil
   -- end
   -- local alt_diff = math.abs(loc:alt()*0.01 - gps_loc:alt()*0.01)
   -- local range_3D_m = math.sqrt(range_2D_m^2 + alt_diff^2)
   -- if range_3D_m < SSIM_DIST_OFS:get() then
   --    return 0
   -- end
   -- local range_ticks = math.floor((range_3D_m - SSIM_DIST_OFS:get()) / SSIM_DIST_MUL:get())
   -- return range_ticks
end

local function send_ranges()
   local range_str = ""
   for i = 1, #SSIM_GND_NODEID do
      local range = get_range_ticks(i)
      if range then
         if #range_str > 0 then
            range_str = range_str .. ","
         end
         range_str = range_str .. string.format([["%u","%u","10"]], SSIM_GND_NODEID[i]:get(), range)
      end
   end
   local json = string.format([[{"result" : [%s],"id" : "sbkb5u0c", "jsonrpc" : "2.0"}]], range_str)
   local msg = string.format([[
Content-Type: application/json
Cache-Control: no-cache
Content-Length: %u
Server: silvus sim

%s]], #json, json)
   send_json(json)
   msg = string.gsub(msg, "\n", "\r\n")
end

--[[
   parse JSON request from remote radio
--]]
local function parse_request()
   --save_to_file("json_rep.txt", http_request)
   for s in http_request:gmatch("[^\r\n]+") do
      table.insert(lines, s)
   end
   local success, req = pcall(json.parse, lines[#lines])
   if not success then
      return
   end
   -- gcs:send_text(0, lines[#lines])
   local method = req['method']
   if not method then
      -- badly formatted
      return
   end

   if method == "current_tof" then
      send_ranges()
      return
   end
   if method == "noise_level" then
      send_noise_level()
      return
   end
   if method == "nbr_rssi" then
      send_nbr_rssi()
      return
   end
   if method == "link_throughput" then
      send_link_throughput()
      return
   end
   if method == "network_status" then
      send_network_status()
      return
   end
   if method == "nbr_mcs" then
      send_mcs()
      return
   end
   if method == "weakest_link" then
      send_weakest_link()
      return
   end
   gcs:send_text(0, "Unknown method (" .. method .. ")")
end

--[[
   handle the "request complete" state; close connection, parse request
--]]
local function finalise_request()
   lines = {}
   if not http_request then
      return
   end
   if not json_log then
      json_log = io.open("json_sim.log",'wb')
   end
   if json_log then
      json_log:write(http_request)
   end

   parse_request()
   connection_sock:close()
   connection_sock = nil
end

--[[
   see if we have a API reply
--]]
local function check_request()
   if not connection_sock then
      -- should not have been called?!
      return
   end
   local now = millis()
   if request_start and now - request_start > REQUEST_TIMEOUT then
      finalise_request()
      return
   end
   local r = connection_sock:recv(1024)
   -- if r == nil then
   --    gcs:send_text(0, "r is nil")
   -- end
   -- if r == "" then
   --    gcs:send_text(0, "r is empty")
   -- end
   -- if r ~= nil then
   --    gcs:send_text(0, "r (" .. r .. ")")
   -- end

   if r == nil or r == "" then
      finalise_request()
      return
   end

   http_request = http_request .. r
end

--[[
   check for new connections
--]]
local function update()
   if SSIM_ENABLE:get() <= 0 then
      return
   end
   if connection_sock then
      check_request()
      return
   end
   if not listen_sock then
      listen_sock = Socket(0)
      if not listen_sock then
         return
      end
      assert(listen_sock:bind("0.0.0.0", SSIM_LISTEN_PORT:get()))
      assert(listen_sock:listen(2))
      listen_sock:reuseaddress()
   end
   connection_sock = listen_sock:accept()
   if not connection_sock then
      return
   end
   connection_sock:set_blocking(true)
   http_request = ''
end

-- wrapper around update(). This calls update() at 20Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(0, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     return protected_wrapper, 1000
  end
  return protected_wrapper, 50
end

return protected_wrapper,100
