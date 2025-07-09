--[[
   monitor silvus radio TOF data and give to AHRS for range fusion
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

PARAM_TABLE_KEY = 46
PARAM_TABLE_PREFIX = "SLV_"

local PORT_HEATBEAT = 8888
local MAX_GROUND_RADIOS = 8

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 63), 'could not add param table')

--[[
  // @Param: SLV_ENABLE
  // @DisplayName: enable Silvus monitor
  // @Description: Enable Silvus monitor
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local SLV_ENABLE = bind_add_param('ENABLE',  1, 1)
if SLV_ENABLE:get() == 0 then
   return
end

local SLV_LOCAL_IP = { bind_add_param('LOCAL_IP0', 2, 192),
                 bind_add_param('LOCAL_IP1', 3, 168),
                 bind_add_param('LOCAL_IP2', 4, 0),
                 bind_add_param('LOCAL_IP3', 5, 2) }

--[[
  // @Param: SLV_RATE
  // @DisplayName: request rate
  // @Description: request rate
  // @Units: Hz
  // @User: Standard
--]]
local SLV_RATE = bind_add_param('RATE', 6, 1)

--[[
  // @Param: SLV_HTTP_PORT
  // @DisplayName: Silvus HTTP port
  // @Description: Silvus HTTP port
  // @Units: m
  // @User: Standard
--]]
local SLV_HTTP_PORT = bind_add_param('HTTP_PORT', 10, 80)

local SLV_LOCAL_NODEID = bind_add_param('LOCAL_NODEID', 14, 0)

local SLV_GND_NODEID = {}

local radio_ranges = {nil, nil}
local radio_tstamp_ms = {nil, nil}

gcs:send_text(MAV_SEVERITY.INFO, "Silvus: starting")

--[[
   get IP address of local radio
--]]
local function local_ip()
   return string.format("%u.%u.%u.%u", SLV_LOCAL_IP[1]:get(), SLV_LOCAL_IP[2]:get(), SLV_LOCAL_IP[3]:get(), SLV_LOCAL_IP[4]:get())
end

local function save_to_file(fname, data)
   local fh = io.open(fname,'wb')
   fh:write(data)
   fh:close()
end

local sock = nil
local http_reply = nil
local reply_start = nil
local REQUEST_TIMEOUT = 250
local last_request_ms = nil
local last_heartbeat_ms = nil
local json = require("json")
local json_log = nil
local handle_response = nil

--[[
   make a silvus API request
--]]
local function http_request(api, params, http_request_response_handler)
   if sock then
      sock:close()
      sock = nil
   end
   sock = Socket(0)
   local node_ip = local_ip()
   if not sock:connect(node_ip, SLV_HTTP_PORT:get()) then
      gcs:send_text(MAV_SEVERITY.ERROR, string.format("Silvus: failed to connect to " .. node_ip .. ":" .. SLV_HTTP_PORT:get(), name))
      sock = nil
      return nil
   end
   sock:set_blocking(true)
   local json = ""
   if params == nil then
      json = string.format([[{"jsonrpc":"2.0","method":"%s","id":"sbkb5u0c"}]], api)
   elseif params.num == 1 then
      json = string.format([[{"jsonrpc":"2.0","method":"%s", "params":["%s"],"id":"sbkb5u0c"}]], api, params.p1)
   elseif params.num == 2 then
      json = string.format([[{"jsonrpc":"2.0","method":"%s", "params":["%s", "%s"],"id":"sbkb5u0c"}]], api, params.p1, params.p2)
   else
      gcs:send_text(MAV_SEVERITY.EMERGENCY,"Error: Unsupported params.")
      return nil
   end
   -- gcs:send_text(MAV_SEVERITY.INFO, "Json: " .. json)
   local cmd = string.format([[POST /streamscape_api HTTP/1.1
Host: %s
User-Agent: lua
Connection: close
Content-Length: %u

]], node_ip, #json)
   cmd = string.gsub(cmd,"\n","\r\n")
   local full_cmd = cmd .. json
   --save_to_file("json_req.txt", full_cmd)
   -- sock:set_blocking(false)
   sock:send(cmd, #cmd)
   sock:send(json, #json)
   http_reply = ''
   reply_start = millis()
   handle_response = http_request_response_handler
end

local function handle_response_noise_level(result)
   gcs:send_named_float("SR_REMNSE", tonumber(result[1]))
end

local function handle_response_throughput(result)
   gcs:send_named_float("SR_REMTPUT", tonumber(result[1]))
end

local function handle_response_rssi(result)
   gcs:send_named_float("SR_RXRSSI1", tonumber(result[1]))
   gcs:send_named_float("SR_RXRSSI2", tonumber(result[2]))
   gcs:send_named_float("SR_RXRSSI3", tonumber(result[3]))
   gcs:send_named_float("SR_RXRSSI4", tonumber(result[4]))
end

local function handle_response_tof(result)
   for res = 1, #result/3 do
      index1 = (res-1)*3+1
      index2 = (res-1)*3+2
      index3 = (res-1)*3+3
      -- gcs:send_text(MAV_SEVERITY.ERROR, "TOFi "..res.. " is "..index1.." "..index2.." "..index3)
      gcs:send_text(MAV_SEVERITY.ERROR, "TOF "..res.. " is "..result[index1].." "..result[index2].." "..result[index3])
      -- logger:write('STOF','n,nid,tof,age','Ifff',res, result[index1],result[index2], result[index3])
   end
end


--[[
   see if we have a API reply, parse it if so
--]]
local function check_reply() 
   if not sock then
      return
   end
   local now = millis()
   if reply_start and now - reply_start > REQUEST_TIMEOUT then
      sock:close()
      sock = nil
      lines = {}
      if not http_reply then
         return
      end
      if not json_log then
         json_log = io.open("json.log",'wb')
      end
      if json_log then
         json_log:write(http_reply)
      end
      --save_to_file("json_rep.txt", http_reply)
      for s in http_reply:gmatch("[^\r\n]+") do
         table.insert(lines, s)
      end
      local success, req = pcall(json.parse, lines[#lines])
      if not success then
         gcs:send_text(MAV_SEVERITY.ERROR, "request failed")
         return
      end
      -- gcs:send_text(MAV_SEVERITY.ERROR, lines[#lines])
      local result = req['result']
      if result == nil then
         gcs:send_text(0, "nil here")
         return
      end
      if not result then
         -- badly formatted
         return
      end
      handle_response(result)
      return
   end
   sock:set_blocking(true)
   local r = sock:recv(1024)
   if r then
      http_reply = http_reply .. r
   end
end


local heartbeat_counter = 0

local http_request_table = {}
http_request_table = { 
   { "noise_level", nil, handle_response_noise_level },
   { "link_throughput", { num=2, p1=SLV_LOCAL_NODEID:get(), p2=1 }, handle_response_throughput },
   { "nbr_rssi", { num=1, p1=SLV_LOCAL_NODEID:get()}, handle_response_rssi },
   { "current_tof", nil, handle_response_tof },
}
local n = 1

--[[
   update called at 20Hz
--]]
local function update()
   if SLV_ENABLE:get() <= 0 then
      return
   end
   if sock then
      check_reply()
      return
   end
   local now = millis()

   if n > #http_request_table then
      -- gcs:send_text(MAV_SEVERITY.INFO, "N is "..n..". Resetting")
      n = 1
   end
   local period_ms = 1000.0 / SLV_RATE:get()
   if not last_request_ms or now - last_request_ms >= period_ms then
      last_request_ms = now
      -- gcs:send_text(MAV_SEVERITY.INFO, "Sending request for n="..n)
      api = http_request_table[n][1]
      params_layout = http_request_table[n][2]
      response_handler = http_request_table[n][3]
      http_request(api, params_layout, response_handler)
      n = n+1
   end
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
