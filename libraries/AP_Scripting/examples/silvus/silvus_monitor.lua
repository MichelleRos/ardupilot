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

local SLV_LOCAL_NODEID = bind_add_param('LOCAL_NODEID', 14, 42)

local LOG_RATE = 0.1 -- once per 10 sec

local SLV_GND_NODEID = {}
local TOF_TABLE = {}
table.insert(TOF_TABLE, {id=SLV_LOCAL_NODEID:get(), tof={-1,-1}, nse={-1,-1}, lt={-1,-1}, rssi = {-1,-1,-1,-1,-1} })
local REQUESTED_NODE = nil

local radio_ranges = {nil, nil}
local radio_tstamp_ms = {nil, nil}

gcs:send_text(MAV_SEVERITY.INFO, "Silvus: starting")


local function nows()
   return millis():toint()
end

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
local last_log_ms = nil
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
   if params.p1 == "RN" then
      params.p1 = REQUESTED_NODE
   end
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
   gcs:send_text(MAV_SEVERITY.INFO, "Json: " .. json)
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
   local noise = tonumber(result[1])
   if REQUESTED_NODE == SLV_LOCAL_NODEID:get() then
      gcs:send_named_float("SR_LOCNSE", noise)
   else
      gcs:send_named_float("SR_REMNSE", noise)
   end
   TOF_TABLE[findTOFidx(REQUESTED_NODE)].nse = { nows(), noise }
end

local function handle_response_throughput(result)
   local link_tput = tonumber(result[1])
   if REQUESTED_NODE == SLV_LOCAL_NODEID:get() then
      gcs:send_named_float("SR_REMTPUT", link_tput)
   else
      gcs:send_named_float("SR_LOCTPUT", link_tput)
   end
   TOF_TABLE[findTOFidx(REQUESTED_NODE)].lt = { nows(), link_tput }
end

local function handle_response_rssi(result)
   local rssi = { tonumber(result[1]), tonumber(result[2]), tonumber(result[3]), tonumber(result[4]) } 
   if REQUESTED_NODE == SLV_LOCAL_NODEID:get() then
      gcs:send_named_float("SR_RXRSSI1", rssi[1])
      gcs:send_named_float("SR_RXRSSI2", rssi[2])
      gcs:send_named_float("SR_RXRSSI3", rssi[3])
      gcs:send_named_float("SR_RXRSSI4", rssi[4])
   else
      gcs:send_named_float("SR_TXRSSI1", rssi[1])
      gcs:send_named_float("SR_TXRSSI2", rssi[2])
      gcs:send_named_float("SR_TXRSSI3", rssi[3])
      gcs:send_named_float("SR_TXRSSI4", rssi[4])
   end
   TOF_TABLE[findTOFidx(REQUESTED_NODE)].rssi = { nows(), rssi[1], rssi[2], rssi[3], rssi[4] }
end

function findTOFidx(val)
   for i, TR in pairs(TOF_TABLE) do
       if TR.id == val then
           return i
       end
   end
   return nil
end

local function handle_response_tof(result)
   -- gcs:send_text(3, "Handling tof "..result[1])
   for res = 1, #result/3 do
      local index1 = (res-1)*3+1
      local index2 = (res-1)*3+2
      local index3 = (res-1)*3+3
      -- gcs:send_text(MAV_SEVERITY.ERROR, "TOFi "..res.. " is "..index1.." "..index2.." "..index3)
      -- gcs:send_text(MAV_SEVERITY.ERROR, "TOF"..res.. " = "..result[index1].." "..result[index2].." "..result[index3])
      -- table.insert(TOF_TABLE, { )
      local idx = tonumber(result[index1])
      if findTOFidx(idx) == nil then
         table.insert(TOF_TABLE, {id=idx, tof={-1,-1}, nse={-1,-1}, lt={-1,-1}, rssi = {-1,-1,-1,-1,-1} })
      end
      TOF_TABLE[findTOFidx(idx)].tof = { result[index3], result[index2]} --always age first, then data
   end
   -- gcs:send_text(MAV_SEVERITY.ERROR, "Finished handling tof")
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

local function log_data()
   gcs:send_text(MAV_SEVERITY.INFO, "In log_data, TOF table is "..#TOF_TABLE)
   for i, TR in pairs(TOF_TABLE) do
      gcs:send_text(MAV_SEVERITY.INFO, "i is "..i)
      gcs:send_text(MAV_SEVERITY.INFO, "Log: TOF: ".. TR.tof[1].." "..TR.tof[2])
      logger:write('STOF','I,ta,t,na,n,la,l,ra,r1,r2,r3,r4','Iiiiiiiiiiii', '#-----------', '------------', i, TR.tof[1], TR.tof[2], TR.nse[1], TR.nse[2], TR.lt[1], TR.lt[2], TR.rssi[1], TR.rssi[2], TR.rssi[3], TR.rssi[4], TR.rssi[5])
   end
end

local heartbeat_counter = 0

local http_request_table = {}
http_request_table = { 
   { "noise_level", nil, handle_response_noise_level },
   { "link_throughput", { num=2, p1="RN", p2=1 }, handle_response_throughput },
   { "nbr_rssi", { num=1, p1="RN"}, handle_response_rssi },
   { "current_tof", nil, handle_response_tof },
}
local n = 0

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

   tot = #TOF_TABLE*3-1

   -- local log_period_ms = 1000.0/LOG_RATE
   -- if not last_log_ms or now - last_log_ms >= log_period_ms then
   --    last_log_ms = now
   --    log_data()
   -- end

   local period_ms = 1000.0 / SLV_RATE:get()
   if not last_request_ms or now - last_request_ms >= period_ms then
      last_request_ms = now
      -- gcs:send_text(MAV_SEVERITY.INFO, "Sending request for n="..n)
      local quo = (n // 3)+1  -- integer division
      local rem = (n % 3)+1
      if n <= tot then
         -- call each http request for each node
         REQUESTED_NODE=TOF_TABLE[quo].id
         gcs:send_text(MAV_SEVERITY.INFO, "RN is "..REQUESTED_NODE.." n is "..n.." tot is "..tot.." tab is "..#TOF_TABLE.." quo is "..quo.." rem is "..rem)
         api = http_request_table[rem][1]
         params_layout = http_request_table[rem][2]
         response_handler = http_request_table[rem][3]
         http_request(api, params_layout, response_handler)
         n = n+1
      else
         -- call TOF and reset counter
         gcs:send_text(MAV_SEVERITY.INFO, "TOF - RN is "..REQUESTED_NODE.." n is "..n.." tot is "..tot.." tab is "..#TOF_TABLE.." quo is "..quo.." rem is "..rem)
         api = http_request_table[4][1]
         params_layout = http_request_table[4][2]
         response_handler = http_request_table[4][3]
         http_request(api, params_layout, response_handler)
         n = 0
      end
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
