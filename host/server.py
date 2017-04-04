import socket
import fifo
import time
import select
from PIL import Image

def main():
    serial_list = b''
    serial_count = 0
    serial_total = 153607
    serial_fifo = fifo.BytesFIFO(640*480*2)
    try:
        sock = socket.create_connection(('192.168.1.3', 23), 10)
        print('Created connection to 192.168.1.3')
        print('Recieving data...')
        while True:
            data = sock.recv(4096)
            if data:
                serial_count += len(data)
                serial_fifo.write(data)
            if serial_count == serial_total:
                print('Recieved whole image')
                display_image(serial_fifo.read(len(serial_fifo)))
                serial_list = b''
                serial_count = 0
                print('reset serial count and list')
    except Exception as e:
        print(serial_count)
        print(e)
        print('Connection failed/EVERYTHING IS ON FIRE')


def display_image(l):
    data_size = (l[l[2]+3]) + (l[l[2]+4] << 8) + (l[l[2]+5] << 16) + (l[l[2]+6] << 24)

    if data_size != 0:
        filename = 'data/output_' + time.strftime("%Y%m%d_%H%M%S", time.gmtime()) + '.raw'
        with open(filename, "wb") as f:
            f.write(bytes(l[l[2]+7:]))

        print("\tSaved image to data folder.")
        print("\tProcessing image for display.")

        f.close()

        g = b''
#        for i in range(1,len(l[l[2]+7:]),2):
        for i in range(0,len(l[l[2]+7:])-1,2):
            g += bytes([l[l[2]+7+i]])

        print("\tDisplaying image.")
        if len(g) != 320*240:
            g += bytes(320*240-len(g))
        print("g is " + str(len(g)) + " bytes")

        im = Image.frombytes("L", (320,240), g)
        im.show()

    else:
        print_error("\tImage sent without data packet!")


if __name__ == '__main__':
    main()
