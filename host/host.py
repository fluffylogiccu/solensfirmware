import serial
import glob
import sys
import threading
import _thread
import queue
import os
import subprocess
import re
import colorama
import atexit
import time
import signal
import struct
import gc
import fifo
from PIL import Image
import socket
import sys

# Module definitions
# KEEP IN SYNC WITH C CODE
log_modules = { 
    0:  'LOG',
    1:  'CMD',
    2:  'STDLIB',
    3:  'SDRAM',
    4:  'OV5642',
    5:  'OV7670',
    6:  'PROF',
    7:  'TEST',
    8:  'CAM',
    9:  'ESP8266',
    10: 'WIFI',
}

# reversed for easier sending
cmd_modules = {
    'LOG':     0,
    'CMD':     1,
    'STDLIB':  2,
    'SDRAM':   3,
    'OV5642':  4,
    'OV7670':  5,
    'PROF':    6,
    'TEST':    7,
    'CAM':     8,
    'ESP8266': 9,
    'WIFI':    10,
}

# Error definitions
# KEEP IN SYNC WITH C CODE
INFO = 0
WARN = 20
ERR  = 40
END  = 60

log_status = {
    'LOG': {
        INFO:   'LOG_INFO_OK',
        WARN-1: 'LOG_INFO_UNKNOWN',
        WARN:   'LOG_WARN_ALINIT',
        WARN+1: 'LOG_WARN_IGNORED',
        ERR-1:  'LOG_WARN_UNKNOWN',
        ERR:    'LOG_ERR_DATASIZE',
        ERR+1:  'LOG_ERR_MSGSIZE',
        ERR+2:  'LOG_ERR_LOGOFF',
        END-1:  'LOG_ERR_UNKNOWN' 
    },
    'CMD': {
        INFO:   'CMD_INFO_OK',
        INFO+1: 'CMD_INFO_INTERRUPT',
        WARN-1: 'CMD_INFO_UNKNOWN',
        WARN:   'CMD_WARN_FREE', 
        WARN+1: 'CMD_WARN_ALINIT',
        ERR-1:  'CMD_WARN_UNKNOWN',
        ERR:    'CMD_ERR_QUEUEEMPTY',
        ERR+1:  'CMD_ERR_QUEUEFULL',
        ERR+2:  'CMD_ERR_QUEUEINVALID',
        ERR+3:  'CMD_ERR_MALLOC',
        ERR+4:  'CMD_ERR_NULLPTR',
        ERR+5:  'CMD_ERR_DATA',
        ERR+6:  'CMD_ERR_NOFUNC',
        ERR+7:  'CMD_ERR_NOMOD',
        END-1:  'CMD_ERR_UNKNOWN'
    },
    'STDLIB': {
        INFO:   'STDLIB_INFO_OK',
        WARN-1: 'STDLIB_INFO_UNKNOWN',
        ERR-1:  'STDLIB_WARN_UNKNOWN',
        END-1:  'STDLIB_ERR_UNKNOWN'
    },
    'SDRAM': {
        INFO:   'SDRAM_INFO_OK',
        WARN-1: 'SDRAM_INFO_UNKNOWN',
        WARN:   'SDRAM_WARN_ALINIT', 
        ERR-1:  'SDRAM_WARN_UNKNOWN',
        END-1:  'SDRAM_ERR_UNKNOWN'
    },
    'OV5642': {
        INFO:   'OV5642_INFO_OK',
        INFO+1: 'OV5642_INFO_IMAGE',
        WARN-1: 'OV5642_INFO_UNKNOWN',
        WARN:   'OV5642_WARN_ALINIT', 
        ERR-1:  'OV5642_WARN_UNKNOWN',
        ERR:    'OV5642_ERR_I2CSTART',
        ERR+1:  'OV5642_ERR_I2CREAD',
        ERR+2:  'OV5642_ERR_I2CWRITE',
        ERR+3:  'OV5642_ERR_I2CTIMEOUT',
        END-1:  'OV5642_ERR_UNKNOWN'
    },
    'OV7670': {
        INFO:   'OV7670_INFO_OK',
        INFO+1: 'OV7670_INFO_IMAGE',
        WARN-1: 'OV7670_INFO_UNKNOWN',
        WARN:   'OV7670_WARN_ALINIT', 
        ERR-1:  'OV7670_WARN_UNKNOWN',
        ERR:    'OV7670_ERR_I2CSTART',
        ERR+1:  'OV7670_ERR_I2CREAD',
        ERR+2:  'OV7670_ERR_I2CWRITE',
        ERR+3:  'OV7670_ERR_I2CTIMEOUT',
        END-1:  'OV7670_ERR_UNKNOWN'
    },
    'PROF': {
        INFO:   'PROF_INFO_OK',
        INFO+1: 'PROF_INFO_RESULTS',
        WARN-1: 'PROF_INFO_UNKNOWN',
        WARN:   'PROF_WARN_ALINIT',
        ERR-1:  'PROF_WARN_UNKNOWN',
        END-1:  'PROF_ERR_UNKNOWN'
        
    },
    'TEST': {
        INFO:   'TEST_INFO_OK',
        INFO+1: 'TEST_INFO_PASSED',
        INFO+2: 'TEST_INFO_CONFIRM',
        WARN-1: 'TEST_INFO_UNKNOWN',
        WARN:   'TEST_WARN_FAILED',
        ERR-1:  'TEST_WARN_UNKNOWN',
        END-1:  'TEST_ERR_UNKNOWN'
    },
    'CAM': {
        INFO:   'CAM_INFO_OK',
        INFO+1: 'CAM_INFO_IMAGE',
        WARN-1: 'CAM_INFO_UNKNOWN',
        WARN:   'CAM_WARN_ALINIT',
        WARN+1:   'CAM_WARN_ALCONF',
        ERR-1:  'CAM_WARN_UNKNOWN',
        ERR:    'CAM_ERR_INIT',
        ERR+1:  'CAM_ERR_CONFIG',
        ERR+2:  'CAM_ERR_CAPTURE',
        ERR+3:  'CAM_ERR_TRANSFER',
        END-1:  'CAM_ERR_UNKNOWN',
    },
    'ESP8266': {
        INFO:   'ESP8266_INFO_OK',
        WARN-1: 'ESP8266_INFO_UNKNOWN',
        ERR-1:  'ESP8266_WARN_UNKNOWN',
        END-1:  'ESP8266_ERR_UNKNOWN'
    },
    'WIFI': {
        INFO:   'WIFI_INFO_OK',
        WARN-1: 'WIFI_INFO_UNKNOWN',
        WARN:   'WIFI_WARN_ALINIT',
        ERR-1:  'WIFI_WARN_UNKNOWN',
        ERR:    'WIFI_ERR_DATASIZE',
        ERR+1:  'WIFI_ERR_MSGSIZE',
        ERR+2:  'WIFI_ERR_SEND',
        ERR+3:  'WIFI_ERR_INIT',
        END-1:  'ESP8266_ERR_UNKNOWN'
    },

}

cmd_functions = {
    'LOG': {
        'LOG_FUNC_INIT': 0, 
    },
    'CMD': {
        'CMD_FUNC_INIT': 0,
    },
    'STDLIB': {
        'STD_FUNC_DUMMY': 0,
    },
    'SDRAM': {
        'SDRAM_FUNC_INIT': 0,
    },
    'OV5642': {
        'OV5642_FUNC_DUMMY': 0,
    },
    'OV7670': {
        'OV7670_FUNC_DUMMY': 0,
    },
    'PROF': {
        'PROF_FUNC_INIT': 0,
    },
    'TEST': {
        'TEST_FUNC_DUMMY': 0,
    },
    'CAM': {
        'CAM_FUNC_INIT':     0,
        'CAM_FUNC_CONFIG':   1,
        'CAM_FUNC_CAPTURE':  2,
        'CAM_FUNC_TRANSFER': 3,
    },
    'ESP8266': {
        'ESP8266_FUNC_DUMMY': 0,
    },
    'WIFI': {
        'WIFI_FUNC_DUMMY': 0,
    }
}

# Serial port Baud rate
BAUD_RATE = 115200 #921600

# Serial port
ser = serial.Serial()

# Serial Rx data
q_list = b''

# Serial command byte count and sizes
serial_count = 0
serial_total = 7
serial_header_size = 3
serial_msg_size = 0
serial_data_size = 0
serial_list = b''
serial_timeout = 10.0
serial_start_time = 0
serial_fifo = fifo.BytesFIFO(640*480*2)
serial_data_flag = 0

# Serial flag to block on transmit
serial_rx = True

# User Input
q_input = queue.Queue()

# st-util process
p = None 

# gdb process
d = None

# wifi socket
sock = None

# Processing threads
tr = None   # User input thread
tp = None   # st-util process thread
ts = None   # Serial thread
tt = None   # Socket thread (TCP connection)

def print_error(string):
    print(colorama.Fore.RED + colorama.Style.BRIGHT + string + colorama.Style.RESET_ALL)

def print_warning(string):
    print(colorama.Fore.MAGENTA + colorama.Style.BRIGHT + string + colorama.Style.RESET_ALL)

def print_info(string):
    print(colorama.Fore.BLUE + colorama.Style.BRIGHT + string + colorama.Style.RESET_ALL)

def print_prompt(string):
    print(colorama.Fore.GREEN + colorama.Style.BRIGHT + string + colorama.Style.RESET_ALL)

def print_welcome():
    print(colorama.Fore.YELLOW + colorama.Style.BRIGHT)
    print("Welcome to the Solens Debug Interface")
    print("-------------------------------------")
    print("        ........ ...                   ")
    print("       ...DI....DD...                  ")
    print("        ..?DDDDDDDDD.                  ")
    print("       .....DD8O...8.                  ")
    print("     .,D... .....=D8D.D$  ..DD=.       ")
    print("     DDN...  .D8DDD~DDD8....Z8D.       ")
    print("    .D8D...  .DDDDDDDDDD....D8DD       ")
    print("     DDDD..  .DZDDD8,D+.D...DDDD       ")
    print("     DDD8D.  .ODDO.~DDDD8..D~DDD       ")
    print("     .D$8DD,=.IDDDDD8DDDDZ.D8DD:       ")
    print("     .8DDDDD?.8DDDDDD,D~D8.DDDD=       ")
    print("     ..DDDDDDD,D$DDDDDDDD..~DD+.       ")
    print("      ..$7DDDD.DDDDD88DDDDD.8D..       ")
    print("       ....,8D$DDD8DD8NDD888:...       ")
    print("          ...DIDDDDDD.. .8DD=          ")
    print("          .....DDD8.+.  .8,D8. .       ")
    print("             .DDDDDDD,..O8DDDO..       ")
    print("          ...8DDDDDD8.DDD:DDDD..       ")
    print("          . .D8...$DDDDDD8D+....       ")
    print("          ..DDDDDDDDDDDDDDDDDI..       ")
    print("          .?DDDDDO:DDDDDDD?O.7..       ")
    print("          ...DDDDDD8..DDDDD$..         ")
    print("          . .DDDDD....DDDDD...         ")
    print("          ..8DD8...  ..DDD.  .         ")
    print("          .... .                       ")
    print("            .. .                       ")
    print("-------------------------------------")
    print(colorama.Style.RESET_ALL)

def init():
    # Create UART serial interface thread
    global ts
    ts = threading.Thread(target=proc_read_serial)
    ts.daemon = True
    ts.start()

    # Create user input thread
    global tr
    tr = threading.Thread(target=proc_read_input)
    tr.daemon = True
    tr.start()

    print_info("Ready for commands. Type 'help' to view a list of commands.")

def open_com():
    global ser
    ports = glob.glob('/dev/tty[A-Za-z]*')
    com_list = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            com_list.append(port)
        except(OSError, serial.SerialException):
            pass

    if com_list == []:
        print_error("Could not find any serial ports.\nPlease check your permissions and try again.")
        return None
    else:
        print_prompt("Please select a COM port.")

    i =  0
    for port in com_list:
        print_prompt(str(i) + ": " + port)
        i += 1

    valid_port = False

    while not valid_port:
        try:
            usr_input = int(input())
            valid_port = True
            if usr_input >= i or usr_input < 0:
                print_warning("Not a valid input. Try again")
                valid_port = False
        except(NameError, SyntaxError, ValueError):
            print_warning("Not a valid input. Try again.")
            valid_port = False
            pass

    ser = serial.Serial(com_list[usr_input], BAUD_RATE, timeout=1.0)
    print_info("Serial port " + ser.name + " opened at " + str(BAUD_RATE) + " Baud.")

    print_info("Creating STLINK-V2 connection.")

    global tp
    tp = threading.Thread(target=proc_stlink)
    tp.daemon = True
    tp.start()

    # Wait for STLINK output
    time.sleep(1)

    print_info("Created STLINK-V2 thread.")

def open_socket():
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('', 8888))
    sock.listen(1)

    global tt
    tt = threading.Thread(target=proc_socket)
    tt.daemon = True
    tt.start()

#######################################
# Thread functions section
def proc_stlink():
    global p
    p = subprocess.Popen(['st-util'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print_info("Starting st-util process with pid " + str(p.pid) + ".") 
    atexit.register(lambda: [p.kill(), p.wait()])

    while True:
        if p.stdout.readable():
            line = p.stdout.readline()
            if not line: continue
            line = "STLINK-V2: " + line.decode("utf-8").strip("\n")
            if re.search("ERROR", line):
                print_error(line)
            elif re.search("WARN", line):
                print_warning(line)
            else:
                print_info(line)
            if re.search("Address already in use", line):
                print_error("Could not open STLINK-V2 connection. There seems to be an existing instance of st-util. Try killing the process.")
                _thread.interrupt_main()
            elif re.search("Couldn't find any ST-Link", line):
                print_error("Could not open STLINK-V2 connection. Is the board plugged in?")
                _thread.interrupt_main()

def proc_read_serial():
    global ser
    global serial_rx
    temp = 0
    while True:
        if serial_rx == True and not ser.in_waiting == 0:
            serial_fifo.write(ser.read(ser.in_waiting))

def proc_read_input():
    while True:
        i = None
        try:
            i = input()
        except(NameError, SyntaxError, ValueError):
            print_warning("Not a valid input. Try again.")
            i = None
            pass
        if not i == None:
            q_input.put(i)

def proc_socket():
    global sock
    while True:
        connection, client_address = sock.accept()
        print_info("Host socket connected to " + str(client_address))
        try:
            while(True):
                data = connection.recv(16)
                if data:
                    serial_fifo.write(data)
        finally:
            connection.close()
            sock.close()

#######################################

#######################################
# Input command section
def cmd_send(module, function, data_len, data):
    if module in cmd_modules:
        m = cmd_modules[module]
    else:
        print_error("Module not in list of modules.")
        return

    if function in cmd_functions[module]:
        f = cmd_functions[module][function]
    else:
        print_error("Function not in list of module functions.")
        return

    m = bytes(struct.pack("<B", m))
    f = bytes(struct.pack("<B", f))
    d = bytes(struct.pack('<Q', data)[0:data_len])
    dl = bytes(struct.pack('<H', data_len))

    global serial_rx
    serial_rx = False

    ser.write(m)
    time.sleep(0.001)
    ser.write(f)
    time.sleep(0.001)
    ser.write(dl)
    time.sleep(0.001)
    if not data_len == 0:
        ser.write(d)

    # give UART interrupt time to add command to queue
    time.sleep(0.050)

    serial_rx = True

def cmd_parse_input(cmd):
    cmd = cmd.lower()
    if cmd == "help":
        cmd_help()
    elif cmd == "stlink restart":
        cmd_stlink_restart()
    elif cmd == "debug start":
        cmd_debug_start()
    elif cmd == "debug stop":
        cmd_debug_stop()
    elif cmd == "debug restart":
        cmd_debug_restart()
    elif cmd == "log init":
        cmd_send("LOG", "LOG_FUNC_INIT", 0, 0)
    elif cmd == "cmd init":
        cmd_send("CMD", "CMD_FUNC_INIT", 0, 0)
    elif cmd == "sdram init":
        cmd_send("SDRAM", "SDRAM_FUNC_INIT", 0, 0)
    elif cmd == "cam init":
        cmd_send("CAM", "CAM_FUNC_INIT", 0, 0)
    elif cmd == "cam config":
        cmd_send("CAM", "CAM_FUNC_CONFIG", 0, 0)
    elif cmd == "cam capture":
        cmd_send("CAM", "CAM_FUNC_CAPTURE", 0, 0)
    elif cmd == "cam transfer":
        cmd_send("CAM", "CAM_FUNC_TRANSFER", 0, 0)
    else:
        print_warning("Invalid command. Type 'help' to view a list of commands")

def cmd_help():
    print_info("The following commands are available:\n" + 
          "\thelp:\t\t\tdisplay the help text\n" + 
          "\tstlink\trestart:\trestart the STLINK connection\n" + 
          "\tdebug\tstart:\t\tstart a debug session\n" +
          "\t\tstop:\t\tstop a debug session\n" +
          "\t\trestart:\trestart a debug session\n" +
          "\tlog\tinit:\t\tinitialize the logger module\n" +
          "\tcmd\tinit:\t\tinitialize the command module\n" +
          "\tsdram\tinit:\t\tinitialize the SDRAM interface\n" +
          "\tcam\tinit:\t\tinitialize the camera module\n" +
          "\tcam\tconfig:\t\tconfigure the camera module to take an image\n" +
          "\tcam\tcapture:\tcapture an image with the camera module\n" +
          "\tcam\ttransfer:\ttransfer an image to the debug interface")

def cmd_stlink_restart():
    global p
    global tp
    p.kill()
    p.wait()
    tp = threading.Thread(target=proc_stlink)
    tp.daemon = True
    time.sleep(0.5)
    tp.start()

def cmd_debug_start():
    # Doesn't need own thread because the process is in a new terminal
    global d
    binaries = os.listdir("bin")
    found_file = False
    for f in binaries:
        if f.lower().endswith(".elf"):
            found_file = True
            d = subprocess.Popen(['gnome-terminal', '-x', 'arm-none-eabi-gdb', "bin/" + f, "-x", "config/.gdbinit"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print_info("Starting arm-none-eabi-gdb process using " + f + " file.")
            break
    
    if found_file == False:
        print_error("Could not find a valid .elf file. Try rebuilding.")
        print_warning("Cancelling debug request.")

def cmd_debug_stop():
    # Kills process started by cmd_debug_start
    global d
    if d == None or not d.poll() == None:
        print_warning("Debug process already stopped.")
    else:
        d.kill()
        d.wait()
        print_info("Killed debug process.")

def cmd_debug_restart():
    cmd_debug_stop()
    time.sleep(1);
    cmd_debug_start()


#######################################

#######################################
# Serial logger section
def serial_reset():
    global serial_msg_size
    global serial_data_size
    global serial_count
    global serial_total
    global serial_list
    global serial_start_time
    global serial_header_size
    global serial_data_flag

    serial_header_size = 3
    serial_data_size = 0
    serial_msg_size = 0
    serial_count = 0
    serial_total = 7
    serial_list = b''
    serial_start_time = 0
    serial_rx = True;
    serial_data_flag = 0

def serial_handle_image(l):
    string = ""
    string += log_modules[l[0]] + ":\t"
    string += log_status[log_modules[l[0]]][l[1]]

    if l[2] != 0:
        string += "\n\t"
        substring = ""

        for char in l[3:3+l[2]]:
            substring += chr(char)
        string += substring

    data_size = (l[l[2]+3]) + (l[l[2]+4] << 8) + (l[l[2]+5] << 16) + (l[l[2]+6] << 24)
    
    print_info(string)

    if data_size != 0:
        filename = 'data/output_' + time.strftime("%Y%m%d_%H%M%S", time.gmtime()) + '.raw'
        with open(filename, "wb") as f:
            f.write(bytes(l[l[2]+7:]))

        print_info("\tSaved image to data folder.")
        print_info("\tProcessing image for display.")

        f.close()

        g = b''
#        for i in range(1,len(l[l[2]+7:]),2):
        for i in range(0,len(l[l[2]+7:])-1,2):
            g += bytes([l[l[2]+7+i]])

        print_info("\tDisplaying image.")
        if len(g) != 320*240:
            g += bytes(320*240-len(g))
        print("g is " + str(len(g)) + "bytes")
        
        im = Image.frombytes("L", (320,240), g)
        im.show()

    else:
        print_error("\tImage sent without data packet!")

def serial_print_log(l):
    serial_reset()
    string = ""
    if l[0] in log_modules:
        string += log_modules[l[0]] + ":\t"
        if l[1] in log_status[log_modules[l[0]]]:
            # Handle image differently
            if log_status[log_modules[l[0]]][l[1]] == "OV5642_INFO_IMAGE" or \
               log_status[log_modules[l[0]]][l[1]] == "OV7670_INFO_IMAGE" or \
               log_status[log_modules[l[0]]][l[1]] == "CAM_INFO_IMAGE":
                serial_handle_image(l)
                return
            string += log_status[log_modules[l[0]]][l[1]]
        elif l[1] >= INFO and l[1] < WARN:
            string += log_status[log_modules[l[0]]][WARN-1]
        elif l[1] >= WARN and l[1] < ERR:
            string += log_status[log_modules[l[0]]][ERR-1]
        else:
            string += log_status[log_modules[l[0]]][END-1]

        if l[2] != 0:
            string += "\n\t"
            substring = ""
            for char in l[3:3+l[2]]:
                substring += chr(char)
            string += substring

        data_size = l[l[2]+3] + l[l[2]+4] << 8 + l[l[2]+5] << 16 + l[l[2]+6] << 24

        if data_size != 0:
            string += "\n\tSent with data packet:\n\t\t"
            substring = ""
            i = 0
            for char in l[l[2]+7:l[2]+7+data_size]:
                i += 1
                substring += "0x{:02x}".format(char) + " "
                if i % 8 == 0:
                    substring += "\n\t\t"
            string += substring
        
        if l[1] >= INFO and l[1] < WARN:
            print_info(string)
        elif l[1] >= WARN and l[1] < ERR:
            print_warning(string)
        else:
            print_error(string)

    else:
        print_error("Recieved log with invalid module ID: " + str(l[0]))
        print_error("Check that the previous call to logger terminated its message with a '\\0'");

def serial_parse_log(d):

    global serial_rx
    
    global serial_count
    global serial_msg_size
    global serial_total
    global serial_list
    global serial_header_size
    global serial_data_size
    global serial_start_time
    global serial_timeout
    global serial_data_flag

    if (serial_data_flag == 1):
        # Just get all data, don't look at it

        serial_start_time = time.time()

        if len(d) >= serial_total - serial_count:
            serial_list += d[0:serial_total - serial_count]
            serial_count += serial_total - serial_count
            serial_print_log(serial_list)
            if (d[serial_total - serial_count:] != ''):
                serial_parse_log(d[serial_total - serial_count:])
        else:
            serial_count += len(d)            
            serial_list += d
    else:

        for data in d:
            serial_count += 1

            serial_start_time = time.time()
        
            serial_list += bytes([data])

            if serial_count == serial_header_size:
                serial_msg_size = data
                serial_total += data

            if serial_count == serial_msg_size + serial_header_size + 1:
                serial_data_size += data
            elif serial_count == serial_msg_size + serial_header_size + 2:
                serial_data_size += (data << 8)
            elif serial_count == serial_msg_size + serial_header_size + 3:
                serial_data_size += (data << 16)
            elif serial_count == serial_msg_size + serial_header_size + 4:
                serial_data_size += (data << 24)
                serial_total += serial_data_size
                serial_data_flag = 1
    
            if serial_count == serial_total:
                serial_print_log(serial_list)

    print(serial_count)
    #PAY NO ATTENTION TO THE HACK BELOW THIS LINE
    if serial_count == 153472 or serial_count == 153510 or serial_count == 153548:
        serial_print_log(serial_list)



#######################################

def main():
    print_welcome()
    open_com()
    open_socket()
    if ser == None:
        return
    init()
    serial_reset()
    
    global serial_written
    global serial_fifo

    # Main processing loop
    while True:
        if not serial_start_time == 0 and time.time() > serial_start_time + serial_timeout:
            serial_reset()
            print_warning("Logger reset after timeout.")
        if not serial_fifo.empty():
            serial_parse_log(serial_fifo.read(len(serial_fifo)))
        if not q_input.empty():
            cmd_parse_input(q_input.get())

if __name__ == "__main__":
    try:
        main()
    except(KeyboardInterrupt):
        try:
            sys.exit(0)
            sock.close()
        except(SystemExit):
            os._exit(0)
