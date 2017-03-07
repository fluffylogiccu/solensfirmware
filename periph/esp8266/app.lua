ip, nm, gw=wifi.sta.getip()
print("Wireless Configuration")
print("IP address: ",ip)
print("Netmask: ",nm)
print("Gateway Address: ", gw,'\n')

function tableEmpty(self)
    for _, _ in pairs(self) do
        return false
    end
    return true
end

q = {}

sck=net.createConnection(net.TCP, 0)

--function sentCallback(sent)  
--    print ("in sentCallback")
--    d = file.read(128)
--    if not d == nil then
--        sck:send(d)
--    end
--    print ("entered sentCallback")
--    if not tableEmpty(q) then
--        print("table has data, calling sck send")
--        sck:send(table.remove(q,1))
--    end
--end
sck:on("connection", function() 
    print ("Connected to TCP server on port "..PORT.." and host "..HOST)
    
    if not tmr.create():alarm(1000, tmr.ALARM_SINGLE, function()
        print ("Fired timer")
        if file.open("img.raw", "r") then
            print ("opened file")
            local function sendChunk()
                local line = file.read(512)
                if line then
                    sck:send(line,sendChunk)
                    print ("sent chunk")
                else
                    print("closing file")
                    file.close()
                    sck:close()
                    collectgarbage()
                end
            end
            sendChunk()
        else
            print ("could not open file")
        end
        
--        print ("calling sentCallback")        
--        sentCallback('')
--        uart.alt(1)
--        uart.setup(0, 9600, 8, uart.PARITY_NONE, uart.STOPBITS_1, 0)
--        uart.on("data", 128, function(data)
--            if tableEmpty(q) then
--                print ("table is empty, inserting data")
--                table.insert(q,d)
--                print ("calling sentCallback")
--                sentCallback('')
--            else
--                print ("table is not empty, inserting data")
--                table.insert(q,d)
--            end
--        end, 0)
    end)
    then
        print ("Could not create timer. UART was not initialized.")
    end
end)
sck:connect(PORT, HOST)
