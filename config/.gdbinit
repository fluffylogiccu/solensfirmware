define reload 
target extended-remote :4242
monitor reset
monitor halt
load
disconnect
target extended-remote :4242
monitor reset
monitor halt
break main
continue
end

reload
