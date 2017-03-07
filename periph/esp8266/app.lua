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

current_size = 0
total_size = 153472 --153728

function sendFile()

    if file.open("temp.raw", "r") then
        --print ("opened file")
        local function sendChunk()
            local line = file.read(512)
            if line then
                sck:send(line,sendChunk)
                --print ("sent chunk")
            else
                --print("closing file")
                file.close()
                sck:close()
                collectgarbage()
            end
        end
        sendChunk()
    else
        sck:send("could not open file")
        sck:close()
        --print ("could not open file")
    end

    current_size = 0
    --file.remove("temp.raw")

end

sck:on("connection", function() 
    print ("Connected to TCP server on port "..PORT.." and host "..HOST)
    
    if not tmr.create():alarm(1000, tmr.ALARM_SINGLE, function()
        file.remove("temp.raw")
        file.open("temp.raw","w")
        file.close()
        current_size = 0

        uart.alt(1)
        uart.setup(0, 9600, 8, uart.PARITY_NONE, uart.STOPBITS_1, 0)
        uart.on("data", 128, function(data)
            if file.open("temp.raw", "a+") then
                file.write(data)
                file.close()
                current_size = current_size + #data
                if current_size >= total_size then
                    tmr.delay(10000)
                    sendFile()
                end
            end
        end, 0)
    end)
    then
        print ("Could not create timer. UART was not initialized.")
    end
end)
sck:connect(PORT, HOST)
