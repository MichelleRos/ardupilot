--[[
   monitor silvus radio data to stream and log
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
local SLV_DEBUG = bind_add_param('DEBUG', 15, 0)

local LOG_RATE = 0.1 -- once per 10 sec

local SLV_GND_NODEID = {}
local LINK_TABLE = {}
table.insert(LINK_TABLE, { id=1 , snr={-1,SLV_LOCAL_NODEID:get(),SLV_LOCAL_NODEID:get(),-1}, nse={-1,-1}, lt={-1,-1}, rssi = {-1,-1,-1,-1,-1}, mcs={ -1,-1} })
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
      gcs:send_text(MAV_SEVERITY.ERROR, string.format("Silvus: failed to connect to " .. node_ip .. ":" .. SLV_HTTP_PORT:get()))
      sock = nil
      return nil
   end
   sock:set_blocking(true)

   local p1 = nil
   if params ~= nil then
      if params.p1 == "RN" then
         p1 = math.floor(REQUESTED_NODE)
      else
         p1 = params.p1
      end
   end
   local json = ""
   if params == nil then
      json = string.format([[{"jsonrpc":"2.0","method":"%s","id":"sbkb5u0c"}]], api)
   elseif params.num == 1 then
      json = string.format([[{"jsonrpc":"2.0","method":"%s", "params":["%s"],"id":"sbkb5u0c"}]], api, p1)
   elseif params.num == 2 then
      json = string.format([[{"jsonrpc":"2.0","method":"%s", "params":["%s", "%s"],"id":"sbkb5u0c"}]], api, p1, params.p2)
   else
      gcs:send_text(MAV_SEVERITY.ERROR,"Error: Unsupported params.")
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

local function send_nvf(nodeid, nvfidloc, nvfidrem, res)
   if type(res) == "number" then
      if nodeid == SLV_LOCAL_NODEID:get() then
         gcs:send_named_float(nvfidloc, res)
      else
         gcs:send_named_float(nvfidrem, res)
      end
   else
      for i = 1, #res do
         if nodeid == SLV_LOCAL_NODEID:get() then
            gcs:send_named_float(nvfidloc..i, res[i])
         else
            gcs:send_named_float(nvfidrem..i, res[i])
         end 
      end
   end
end

-- returns LINK_TABLE index number for the given idx
local function findLINKibyidx(idx)
   for i, TR in pairs(LINK_TABLE) do
      if TR.idx == idx then
         return i
      end
   end
   return nil
end

--returns LINK_TABLE index number where the given node id is in the first spot
local function findLINKibynid(nid)
   for i, TR in pairs(LINK_TABLE) do
      if TR.snr[2] == nid then
         return i
      end
   end
   return nil
end

--returns LINK_TABLE index number where the given node id is in the second spot
local function findLINKibynid2(nid)
   for i, TR in pairs(LINK_TABLE) do
      if TR.snr[3] == nid then
         return i
      end
   end
end

local function getnids()
   local hash = {}
   local res = {}
   for _,v in pairs(LINK_TABLE) do
      n1 = v.snr[2]
      n2 = v.snr[3]
      if (not hash[n1]) then
         res[#res+1] = n1
         hash[n1] = true
      end
      if (not hash[n2]) then
         res[#res+1] = n2
         hash[n2] = true
      end
   end
   return res
end

local function debug_msg(sev, msg)
   if SLV_DEBUG:get() == 1 then
      if sev == 1 then
         gcs:send_text(MAV_SEVERITY.INFO, msg)
      elseif sev == 2 then
         gcs:send_text(MAV_SEVERITY.WARNING, msg)
      else 
         gcs:send_text(MAV_SEVERITY.EMERGENCY, msg)
      end
   end
end

local function handle_response_noise_level(result)
   local noise = tonumber(result[1])
   send_nvf(REQUESTED_NODE, "SR_LOCNSE", "SR_REMNSE", noise)
   LINK_TABLE[findLINKibynid(REQUESTED_NODE)].nse = { nows(), noise }
end

local function handle_response_throughput(result)
   local link_tput = tonumber(result[1])
   send_nvf(REQUESTED_NODE, "SR_LOCTPUT", "SR_REMTPUT", link_tput)
   LINK_TABLE[findLINKibynid(REQUESTED_NODE)].lt = { nows(), link_tput }
end

local function handle_response_rssi(result)
   local rssi = { tonumber(result[1]), tonumber(result[2]), tonumber(result[3]), tonumber(result[4]) } 
   send_nvf(REQUESTED_NODE, "SR_RXRSSI", "SR_TXRSSI", rssi)
   LINK_TABLE[findLINKibynid(REQUESTED_NODE)].rssi = { nows(), rssi[1], rssi[2], rssi[3], rssi[4] }
end

local function handle_response_mcs(result)
   local mcs = tonumber(result[1])
   send_nvf(REQUESTED_NODE, "SR_LOCMCS", "SR_REMMCS", mcs)
   LINK_TABLE[findLINKibynid(REQUESTED_NODE)].mcs = { nows(), mcs }
end

local function handle_response_network_status(result)
   for res = 1, #result/3 do
      local nid1i = (res-1)*3+1
      local nid2i = (res-1)*3+2
      local snri = (res-1)*3+3
      local nid1 = tonumber(result[nid1i])
      local nid2 = tonumber(result[nid2i])
      local snr = tonumber(result[snri])
      --add new item if needed
      local idx1 = nid1.." "..nid2
      if findLINKibyidx(idx1) == nil then
         table.insert(LINK_TABLE, { idx=idx1, snr={-1,nid1,nid2,-1}, nse={-1,-1}, lt={-1,-1}, rssi={-1,-1,-1,-1,-1}, mcs={ -1,-1} })
      end
      --always age first, then data
      LINK_TABLE[findLINKibyidx(idx1)].snr = { nows(),nid1, nid2, snr }
      -- nid has been added, but in the second spot.
      if findLINKibynid(nid2) == nil and findLINKibynid2(nid2) ~= nil then
         local idx2 = nid2.." "..nid1
         table.insert(LINK_TABLE, { idx=idx2, snr={-1,nid2,nid1,-1}, nse={-1,-1}, lt={-1,-1}, rssi={-1,-1,-1,-1,-1}, mcs={ -1,-1} })
         debug_msg(2, "Added idx2 item: "..idx2)
      end
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
      if type(req) ~= "table" then
         save_to_file("json_rep.txt", http_reply)
         if type(req) == "string" or type(req) == "number" then
            gcs:send_text(MAV_SEVERITY.ERROR, "Error: Request returned "..req)
         else
            gcs:send_text(MAV_SEVERITY.ERROR, "Error: Request returned a "..type(req))
         end
         return
      end
      debug_msg(1, "Reply is "..lines[#lines])
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
   debug_msg(1,"In log_data, LINK table is "..#LINK_TABLE.." long")
   for i, TR in pairs(LINK_TABLE) do
      -- gcs:send_text(MAV_SEVERITY.INFO, "i is "..i)
      logger:write('SLV1','I,sa,sl,sr,s,na,n,la,l','Iffffffff', '#--------', '---------', i, TR.snr[1], TR.snr[2], TR.snr[3], TR.snr[4], TR.nse[1], TR.nse[2], TR.lt[1], TR.lt[2])
      logger:write('SLV2','I,ra,r1,r2,r3,r4,ma,m','Ifffffff', '#-------', '--------', i, TR.rssi[1], TR.rssi[2], TR.rssi[3], TR.rssi[4], TR.rssi[5],TR.mcs[1],TR.mcs[2])
   end
end

local heartbeat_counter = 0


local http_request_table = {}
http_request_table = { 
   -- api          params   response handler          local remote
   { "noise_level", nil, handle_response_noise_level, true,  false },
   { "link_throughput", { num=2, p1="RN", p2=1 }, handle_response_throughput, true, true },
   { "nbr_rssi", { num=1, p1="RN"}, handle_response_rssi, true, true },
   { "nbr_mcs", { num=1, p1="RN"}, handle_response_mcs, true, true },
   { "network_status", nil, handle_response_network_status, true, false },
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

   NIDS = getnids()
   tot = #NIDS*#http_request_table-1
   if n > tot then
      n = 0
   end

   local log_period_ms = 1000.0/LOG_RATE
   if not last_log_ms or now - last_log_ms >= log_period_ms then
      last_log_ms = now
      log_data()
   end

   local period_ms = 1000.0 / SLV_RATE:get()
   if not last_request_ms or now - last_request_ms >= period_ms then
      last_request_ms = now
      -- gcs:send_text(MAV_SEVERITY.INFO, "Sending request for n="..n)
      local quo = (n // #http_request_table)+1  -- integer division
      local rem = (n % #http_request_table)+1
      -- call each http request for each node
      local ftn = findLINKibynid(NIDS[quo])
      -- check that the nid exists in LINK table before calling it.
      if ftn == nil then
         gcs:send_text(MAV_SEVERITY.WARNING, "NIDS["..quo.."] is "..NIDS[quo].." findLINKibynid=nil")
         for i=1, #NIDS do
            debug_msg("Quo is "..quo.." NIDS["..i.."] is "..NIDS[i])
         end
         last_request_ms = now - period_ms
         n = n+1
         return
      end
      REQUESTED_NODE=NIDS[quo]
      local do_local = (http_request_table[rem][4] and (REQUESTED_NODE == SLV_LOCAL_NODEID:get()))
      local do_remote = (http_request_table[rem][5] and (REQUESTED_NODE ~= SLV_LOCAL_NODEID:get()))
      if do_local or do_remote then
         local api = http_request_table[rem][1]
         local params_layout = http_request_table[rem][2]
         local response_handler = http_request_table[rem][3]
         debug_msg(1, "RN is "..REQUESTED_NODE.." for "..api.." n is "..n.." tot is "..tot.." tab is "..#NIDS.." quo is "..quo)
         http_request(api, params_layout, response_handler)
      else
         local api = http_request_table[rem][1]
         debug_msg(1, "SKIPPED - RN is "..REQUESTED_NODE.." for "..api.." n is "..n.." tot is "..tot.." tab is "..#NIDS.." quo is "..quo)
         last_request_ms = now - period_ms --make sure it gets called again soon
      end
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
