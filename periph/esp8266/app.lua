ip, nm, gw=wifi.sta.getip()
print("Wireless Configuration")
print("IP address: ",ip)
print("Netmask: ",nm)
print("Gateway Address: ", gw,'\n')

data_sent = 0;
d = ''

sck=net.createConnection(net.TCP, 0) 
sck:on("sent", function()
    if #d > 0 then
        sck:send(d)
        d = ''
    else 
        data_sent = 0
    end
end)
--tmr.alarm(0, 10, tmr.ALARM_AUTO, function()
--	if data_size ~= 0 and data_size == #d then
--		sck:send(data)
--		data = ''
--		data_size = 0
--	else 
--		data_size = #d
-- 	end
--end)

uart.alt(1)
uart.setup(0, 9600, 8, uart.PARITY_NONE, uart.STOPBITS_1, 0)
uart.on("data", 128, function(data)
    d = d..data
    if #d > 0 then 
        sck:send(d)
        d = ''
        data_sent = 1
    end

    --if #data > 1024 then
    --    sck:send(data)
    --    data = ''
    --end
end, 0)

sck:connect(PORT,HOST)
