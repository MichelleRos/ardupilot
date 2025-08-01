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
local SLV_RATE = bind_add_param('REQ_RATE', 6, 1)

--[[
  // @Param: SLV_HTTP_PORT
  // @DisplayName: Silvus HTTP port
  // @Description: Silvus HTTP port
  // @Units: m
  // @User: Standard
--]]
local SLV_HTTP_PORT = bind_add_param('HTTP_PORT', 10, 80)

local SLV_LOCAL_NODEID = bind_add_param('LOCAL_NODEID', 14, 42)
local SLV_INFO = bind_add_param('INFO', 15, 2) -- 1 is mostly just warnings, 2 adds max's & weaklink as gcs send text, 3 is debug
-- 16 was log rate
local SLV_NVF_RATE = bind_add_param('NVF_RATE', 17, -1) -- max rate to send nvf for any message at. -1 means no restriction
local SLV_REQ_TIMEOUT = bind_add_param('REQ_TIMEOUT', 18, 250) -- request timeout in milliseconds
local SLV_DEST_NODEID = bind_add_param('DEST_NODEID', 19, 43) -- destination node id for weakest_link

gcs:send_text(MAV_SEVERITY.INFO, "Silvus: Starting")

local sock = nil
local http_reply = nil
local reply_start = nil
local REQUEST_TIMEOUT = 250
local last_request_ms = nil
local last_nvf_ms = nil
local last_flush_ms = nil
local json = require("json")
local handle_response = nil
local NODEID_TABLE = { math.floor(SLV_LOCAL_NODEID:get()), math.floor(SLV_DEST_NODEID:get()) }
local REQUESTED_NODE = nil
local REQUESTED_API = nil
local json_log = nil
local NODE_NAMES = require("nodes")

--[[
   get IP address of local radio
--]]
local function local_ip()
   return string.format("%u.%u.%u.%u", SLV_LOCAL_IP[1]:get(), SLV_LOCAL_IP[2]:get(), SLV_LOCAL_IP[3]:get(), SLV_LOCAL_IP[4]:get())
end

--When info is 3. 1 = emergency, 2 = warning, 3 = info
local function info3_msg(sev, msg)
   if SLV_INFO:get() > 2 then
      if sev == 1 then
         gcs:send_text(MAV_SEVERITY.EMERGENCY, "SilvusE: "..msg)
      elseif sev == 2 then
         gcs:send_text(MAV_SEVERITY.WARNING, "SilvusW: "..msg)
      else 
         gcs:send_text(MAV_SEVERITY.INFO, "SilvusI: "..msg)
      end
   end
end

-- when info is 2 or 3
local function info2_msg(msg)
   if SLV_INFO:get() > 1 then
      gcs:send_text(MAV_SEVERITY.INFO, "SilvusI: "..msg)
   end
end

--When info is 1, 2 or 3. 1 = emergency, 2 = warning, 3 = info
local function info1_msg(sev, msg)
   if SLV_INFO:get() > 0 then
      if sev == 1 then
         gcs:send_text(MAV_SEVERITY.EMERGENCY, "SilvusE: "..msg)
      elseif sev == 2 then
         gcs:send_text(MAV_SEVERITY.WARNING, "SilvusW: "..msg)
      else 
         gcs:send_text(MAV_SEVERITY.INFO, "SilvusI: "..msg)
      end
   end
end

-- just checks whether nid is in the table, returns true if it is, false if not
local function checknidintable(nid)
   for i = 1, #NODEID_TABLE do
      if NODEID_TABLE[i] == nid then
         return true
      end
   end
   return false
end

local function nidname(nid)
   local nam = NODE_NAMES[nid]
   if nam == nil then
      nam = nid
      -- don't add to nidtable here
      if not checknidintable(nid) then
         -- only send message
         info1_msg(2,"Node ID "..nid.." was not expected.")
      end
   end
   return nam
end

-- returns true if nid already in NODEID_TABLE, else it also adds nid to the table and returns false.
local function checkaddnidintable(nid)
   if checknidintable(nid) then
      return true
   end
   info3_msg(2, "Seen new node: "..nid.."("..nidname(nid)..")")
   -- add new item
   table.insert(NODEID_TABLE, nid)
   return false
end

local function send_nvf(nodeid, nvfidloc, nvfidrem, res)
   local now = millis()
   if SLV_NVF_RATE:get() > 0 then
      local nvf_period_ms = 1000.0/SLV_NVF_RATE:get()
      if (last_nvf_ms ~= nil) and ((now - last_nvf_ms) < nvf_period_ms) then
         if nodeid == SLV_LOCAL_NODEID:get() then
            info3_msg(2, "NVF: Too soon, not sending "..nvfidloc)
         else
            info3_msg(2, "NVF: Too soon, not sending "..nvfidrem)
         end
         return
      end
   end
   last_nvf_ms = now

   if type(res) == "number" then
      if nodeid == SLV_LOCAL_NODEID:get() then
         -- info2_msg(3, "NVF: Sending ".. nvfidloc)
         gcs:send_named_float(nvfidloc, res)
      else
         -- info2_msg(3, "NVF: Sending ".. nvfidrem)
         gcs:send_named_float(nvfidrem, res)
      end
   else
      for i = 1, #res do
         if nodeid == SLV_LOCAL_NODEID:get() then
            -- info2_msg(3, "NVF: Sending ".. nvfidloc..i)
            gcs:send_named_float(nvfidloc..i, res[i])
         else
            -- info2_msg(3, "NVF: Sending ".. nvfidrem..i)
            gcs:send_named_float(nvfidrem..i, res[i])
         end 
      end
   end
end

local function send_nvf_single(nvfid, res)
   send_nvf(SLV_LOCAL_NODEID:get(), nvfid, "SR_x", res)
end

local function save_to_json_rep(data)
   local json_rep = io.open("json_rep.txt",'wb')
   if not json_rep then
      info1_msg(1, "Save_to_file's file open failed")
      return
   end
   json_rep:write(data)
   json_rep:close()
end

local function get_next_log_name()
   local nln = io.open("scripts/nextlogno.txt",'r')
   if not nln then
      info1_msg(2, "Couldn't open nextlogno.txt. Using json.log as logname")
      return "json.log"
   end
   local nln_contents = nln:read("*a")
   local logno = tonumber(nln_contents)
   nln:close()
   nln = io.open("scripts/nextlogno.txt",'w')
   if not nln or not logno then
      info1_msg(2, "Couldn't write to nextlogno.txt. Using json.log as logname")
      return "json.log"
   end
   nln:write(logno+1)
   nln:close()
   local logname = "jsonlogs/json"..logno..".log"
   info1_msg(3, "Using "..logname.." as logname")
   return logname
end

local function log_to_json(data)
   if not json_log then
      local logname = get_next_log_name()
      json_log = io.open(logname,'wb')
   end
   if not json_log then
      info1_msg(1, logname.."'s file open failed")
      return
   end
   json_log:write(data)
end

-- make a silvus API request
local function http_request(api, params, http_request_response_handler)
   if sock then
      sock:close()
      sock = nil
   end
   sock = Socket(0)
   local node_ip = local_ip()
   if not sock:connect(node_ip, SLV_HTTP_PORT:get()) then
      info1_msg(1,"Failed to connect to " .. node_ip .. ":" .. math.floor(SLV_HTTP_PORT:get()))
      sock:close()
      sock = nil
      return nil
   end
   sock:set_blocking(true)

   local p1 = nil
   if params ~= nil then
      if params.p1 == "RN" then
         p1 = math.floor(REQUESTED_NODE)
         log_to_json("\nREQUESTED NODE is: "..REQUESTED_NODE.."("..nidname(REQUESTED_NODE)..")\n")
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
      info1_msg(1,"Unsupported params.")
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
   -- sock:set_blocking(false)
   sock:send(cmd, #cmd)
   sock:send(json, #json)
   log_to_json("\nHTTP_REQUEST_SENT:\n"..cmd..json.."\n")
   http_reply = ''
   reply_start = millis()
   handle_response = http_request_response_handler
end

local function handle_response_noise_level(result)
   if #result  ~= 1 then
      info1_msg(2,"Noise level expects #result = 1. #result is "..#result)
      return
   end
   local noise = tonumber(result[1])
   logger:write('SLNL','I,nid,noise','Nif', '#--', '---', nidname(REQUESTED_NODE), REQUESTED_NODE, noise)
   send_nvf(REQUESTED_NODE, "SR_LOCNSE", "SR_REMNSE", noise)
end

local function handle_response_throughput(result)
   if #result  ~= 1 then
      info1_msg(2,"Link throughput expects #result = 1. #result is "..#result.." RN="..REQUESTED_NODE)
      return
   end
   local link_tput = tonumber(result[1])
   logger:write('SLLT','I,nid,ltput','Nif', '#--', '---', nidname(REQUESTED_NODE), REQUESTED_NODE, link_tput)
   send_nvf(REQUESTED_NODE, "SR_LOCTPUT", "SR_REMTPUT", link_tput)
end

local function handle_response_rssi(result)
   if #result  ~= 4 then
      info1_msg(2,"NBR RSSI expects #result = 4. #result is "..#result.." RN="..REQUESTED_NODE)
      return
   end
   local rssi = { tonumber(result[1]), tonumber(result[2]), tonumber(result[3]), tonumber(result[4]) } 
   logger:write('SLNR','I,nid,r1,r2,r3,r4','Niffff', '#-----', '------', nidname(REQUESTED_NODE), REQUESTED_NODE, rssi[1], rssi[2], rssi[3], rssi[4])
   send_nvf(REQUESTED_NODE, "SR_LRSSI", "SR_RRSSI", rssi)
end

local function handle_response_mcs(result)
   if #result  ~= 1 then
      info1_msg(2,"NBR MCS expects #result = 1. #result is "..#result.." RN="..REQUESTED_NODE)
      return
   end
   local mcs = tonumber(result[1])
   logger:write('SLNM','I,nid,mcs','Nif', '#--', '---', nidname(REQUESTED_NODE), REQUESTED_NODE, mcs)
   send_nvf(REQUESTED_NODE, "SR_LOCMCS", "SR_REMMCS", mcs)
end

local function handle_response_weakest_link(result)
   if #result  ~= 4 then
      info1_msg(2,"Weakest Link expects #result = 4. #result is "..#result.." NODE is "..math.floor(SLV_DEST_NODEID:get()))
      return
   end
   local wl1 = math.floor(result[1])
   local wl2 = math.floor(result[2])
   local snr = tonumber(result[3])
   local reuse = tonumber(result[4])
   local wl1n = nidname(wl1)
   local wl2n = nidname(wl2)
   local logid = wl1n.."_"..wl2n
   logger:write('SLWL','I,wl1n,wl1,wl2n,wl2,snr,reuse','NNiNiff', '#------', '-------', logid ,wl1n, wl1, wl2n, wl2, snr, reuse)
   send_nvf_single("SR_WKLKSNR", snr)
   send_nvf_single("SR_WKLKID1", wl1)
   send_nvf_single("SR_WKLKID2", wl2)
   send_nvf_single("SR_WKLKRU", reuse)
   info2_msg("Weaklink SNR="..snr.." Node1="..wl1n.." Node2="..wl2n.." RU="..reuse)
   checkaddnidintable(wl1)
   checkaddnidintable(wl2)
end

local function handle_response_network_status(result)
   if (#result % 3) ~= 0 then
      info1_msg(2,"Network status expects #result divisible by 3. #result is "..#result)
      return
   end
   local max_snr1 = -1
   local max_snr2 = -1
   local max_snr1_nid1 = -1
   local max_snr1_nid2 = -1
   local max_snr2_nid1 = -1
   local max_snr2_nid2 = -1
   for res = 1, #result/3 do
      local nid1i = (res-1)*3+1
      local nid2i = (res-1)*3+2
      local snri = (res-1)*3+3
      local nid1 = math.floor(result[nid1i])
      local nid2 = math.floor(result[nid2i])
      local snr = tonumber(result[snri])
      local idx1 = nidname(nid1).."_"..nidname(nid2)
      logger:write('SLNS','I,nid1,nid2,snr','Niif', '#---', '----', idx1, nid1, nid2, snr)

      -- fill the table to keep track of which nodes to request from
      checkaddnidintable(nid1)
      checkaddnidintable(nid2)

      -- update max calculations
      if snr > max_snr2 and (nid1 == SLV_LOCAL_NODEID:get() or nid2 == SLV_LOCAL_NODEID:get()) then
         max_snr2 = snr
         max_snr2_nid1 = nid1
         max_snr2_nid2 = nid2
      end
      if max_snr2 > max_snr1 then
         -- swap snrs
         local max_snr_tmp = max_snr2
         local max_snr_nid1_tmp = max_snr2_nid1
         local max_snr_nid2_tmp = max_snr2_nid2
         max_snr2 = max_snr1
         max_snr2_nid1 = max_snr1_nid1
         max_snr2_nid2 = max_snr1_nid2
         max_snr1 = max_snr_tmp
         max_snr1_nid1 = max_snr_nid1_tmp
         max_snr1_nid2 = max_snr_nid2_tmp
      end
   end
   -- send max SNRs
   send_nvf_single("SR_M1_SNR", max_snr1)
   send_nvf_single("SR_M1_NID1", max_snr1_nid1)
   send_nvf_single("SR_M1_NID2", max_snr1_nid2)
   send_nvf_single("SR_M2_SNR", max_snr2)
   send_nvf_single("SR_M2_NID1", max_snr2_nid1)
   send_nvf_single("SR_M2_NID2", max_snr2_nid2)
   info2_msg("Max1 SNR="..max_snr1.." Node1="..nidname(max_snr1_nid1).." Node2="..nidname(max_snr1_nid2))
   info2_msg("Max2 SNR="..max_snr2.." Node1="..nidname(max_snr2_nid1).." Node2="..nidname(max_snr2_nid2))
   if SLV_INFO:get() > 2 then
      local tab = ""
      for i = 1, #NODEID_TABLE do
         tab = tab.." "..NODEID_TABLE[i].."("..nidname(NODEID_TABLE[i])..")"
      end
      info3_msg(2, "Seen "..#NODEID_TABLE.." nodes:"..tab)
   end
end

-- see if we have a API reply, parse it if so
local function check_reply() 
   if not sock then
      return
   end
   local now = millis()
   if reply_start and now - reply_start > SLV_REQ_TIMEOUT:get() then
      sock:close()
      sock = nil
      lines = {}
      if not http_reply then
         info1_msg(2,"No http reply")
         return
      end
      log_to_json("\nHTTP_REPLY_RECEIVED:\n"..http_reply.."\n")
      local json_body = ""
      local matching = false
      -- loop through each line in reply
      for s in http_reply:gmatch("[^\r\n]+") do
         -- check for the start of the table
         if not matching and s:find('{') then
            matching = true
         end
         -- filter out hex numbers in between lines
         if matching and not tonumber(s,16) then
            json_body = json_body .. s
         end
      end
      log_to_json("\nJSON_BODY: "..json_body.."\n")
      local success, rep = pcall(json.parse, json_body)
      if not success then
         info1_msg(2,"Json parse failed.")
         info3_msg(2, "JPF json_body is "..json_body)
         log_to_json("\nAbove reply was not parsed successfully.\n")
         save_to_json_rep(http_reply)
         return
      end
      if type(rep) ~= "table" then
         if type(rep) == "string" or type(rep) == "number" then
            info1_msg(2,"Reply is "..rep.." RN="..REQUESTED_NODE)
         else
            info1_msg(2,"Reply is a "..type(rep).." RN="..REQUESTED_NODE)
         end
         save_to_json_rep(http_reply)
         return
      end
      local result = rep['result']
      if result == nil then
         info1_msg(1,"Nil for result")
         save_to_json_rep(http_reply)
         return
      end
      if not result then
         info1_msg(1,"No result")
         save_to_json_rep(http_reply)
         return
      end
      if type(result) ~= "table" then
         info1_msg(2,"Result from reply is not a table. RN="..REQUESTED_NODE)
         save_to_json_rep(http_reply)
         return
      end
      if handle_response == nil then
         info1_msg(1,"Nil for handle_response")
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

local http_request_table = {}
http_request_table = { 
   -- api          params   response handler          local/single remote
   { "network_status", nil, handle_response_network_status, true, false },
   { "noise_level", nil, handle_response_noise_level, true,  false },
   { "link_throughput", { num=2, p1="RN", p2=1 }, handle_response_throughput, true, true },
   { "nbr_rssi", { num=1, p1="RN"}, handle_response_rssi, false, true },
   { "nbr_mcs", { num=1, p1="RN"}, handle_response_mcs, true, true },
   { "weakest_link", { num=1, p1=math.floor(SLV_DEST_NODEID:get())}, handle_response_weakest_link, true, false },
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

   tot = #NODEID_TABLE*#http_request_table-1
   if n > tot then
      n = 0
   end

   local flush_period_ms = 5000.0
   if json_log and (not last_flush_ms or now - last_flush_ms >= flush_period_ms) then
      last_flush_ms = now
      info3_msg(3, "Flushing json log")
      json_log:flush()
   end

   local period_ms = 1000.0 / SLV_RATE:get()
   if not last_request_ms or now - last_request_ms >= period_ms then
      last_request_ms = now
      -- gcs:send_text(MAV_SEVERITY.INFO, "Sending request for n="..n)
      local quo = (n // #http_request_table)+1  -- integer division
      local rem = (n % #http_request_table)+1
      -- call each http request for each node
      local do_local_or_single = (http_request_table[rem][4] and (NODEID_TABLE[quo] == SLV_LOCAL_NODEID:get()))
      local do_remote = (http_request_table[rem][5] and (NODEID_TABLE[quo] ~= SLV_LOCAL_NODEID:get()))
      if do_local_or_single or do_remote then
         local api = http_request_table[rem][1]
         local params_layout = http_request_table[rem][2]
         local response_handler = http_request_table[rem][3]
         REQUESTED_NODE=NODEID_TABLE[quo]
         REQUESTED_API=http_request_table[rem][1]
         info3_msg(3, "RN="..REQUESTED_NODE.."("..nidname(REQUESTED_NODE)..") API="..api.." n="..n.." tot="..tot.." tab="..#NODEID_TABLE.." quo="..quo)
         http_request(api, params_layout, response_handler)
      else
         -- local api = http_request_table[rem][1]
         -- info3_msg(3, "SKIPPED - RN="..REQUESTED_NODE.."("..nidname(REQUESTED_NODE)..") API="..api.." n="..n.." tot="..tot.." tab="..#NODEID_TABLE.." quo="..quo)
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
