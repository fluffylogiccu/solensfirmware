-- define   SSID
--          PASSWORD
--          APPLICATION
dofile("config.lua")

function startup()
    if file.open("init.lua") == nil then
        print("init.lua deleted.")
    else
        print("Exiting initialization script and running application.")
        file.close("init.lua")
        dofile(APPLICATION)
    end
end

wifi.sta.disconnect()

print("Setting up wifi...")
wifi.setmode(wifi.STATION)
wifi.sta.config(SSID,PASSWORD)
tmr.alarm(1, 15000, 1, function()
    if wifi.sta.getip() == nil then
        print("IP unavailable. Check your configuration file.")
        tmr.stop(1)
    else
        tmr.stop(1)
        print("Configured with IP: "..wifi.sta.getip())
        print("Waiting 5 seconds before startup.")
        print("Kill with 'file.remove(\"init.lua\")'.")
        tmr.alarm(0, 5000, 0, startup)
    end
end)
