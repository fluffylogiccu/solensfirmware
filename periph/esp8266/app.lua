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
sck:on("connection", function() 
    print ("Connected to TCP server on port "..PORT.." and host "..HOST)
    
    if not tmr.create():alarm(1000, tmr.ALARM_SINGLE, function()
        uart.alt(1)
        uart.setup(0, 9600, 8, uart.PARITY_NONE, uart.STOPBITS_1, 0)
        uart.on("data", 128, function(data)
        --    sck:send('uart transfer rec ')
        --    uart.write(0, 'Z')
--uart.on("data", 128, function(data)
            
            -- TODO
            -- need to put some sort of lock for sending
            -- LOCK
            -- sck:send(d)
            -- d = ''
            -- UNLOCK
            -- because this way can call a send
            -- get more uart and concat
            -- set d = ''
            -- then we have lost data
            d = d..data
            if #d > 0 then                
                sck:send(d)
                d = ''
                data_sent = 1
            end
        --    sck:send(data);
        end, 0)
    end)
    then
        print ("Could not create timer. UART was not initialized.")
    end
end)
sck:connect(PORT, HOST)
