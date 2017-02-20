ESP8266 configuration

Download esptool at https://github.com/espressif/esptool
to flash firmware binary. 

You can create a firmware binary using the cloud tool at 
https://nodemcu-build.com/

Use nodemcu-uploader or luatool to get lua code onto the 
board. You can find these at:
https://github.com/kmpm/nodemcu-uploader
https://github.com/4refr0nt/luatool

Lua application code should be in a a file app.lua. This 
file will be included by the init.lua script in this folder. 
Configuration variables should be set in the config.lua
script. All of files needto be uploaded to the board with 
the nodmcu-uploader tool.
