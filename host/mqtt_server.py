import paho.mqtt.client as mqtt
import fifo
import time
import select
from PIL import Image

serial_list = b''
serial_count = 0
serial_fifo = fifo.BytesFIFO(640*480*2)

def main():

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect("localhost", 1883, 60)
    try:
        client.loop_forever()
    except (KeyboardInterrupt, SystemExit) as e:
        print(e)
        print('Disconnecting from broker')
        client.disconnect()
    except Exception as e:
        print(e)

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe('img')
    client.subscribe('imgstart')
    client.subscribe('imgend')

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global serial_list
    global serial_fifo
    global serial_count
    if(msg.topic == 'imgstart'):
        print('Start image recieved')
        print(msg.payload)
        serial_list = b''
        serial_count = 0
        serial_fifo.read(len(serial_fifo))
    elif(msg.topic == 'imgend'):
        print('Recieved whole image')
        #display_image(serial_fifo.read(len(serial_fifo)))
        print(len(serial_fifo))
        print(time.strftime("%Y%m%d_%H%M%S", time.gmtime()))
    elif(msg.topic == 'img'):
        serial_count += len(msg.payload)
        serial_fifo.write(msg.payload)


def display_image(l):
    print(len(l))
    filename = 'data/output_' + time.strftime("%Y%m%d_%H%M%S", time.gmtime()) + '.raw'
    pngname =  'data/output_' + time.strftime("%Y%m%d_%H%M%S", time.gmtime()) + '.png'

    with open(filename, "wb") as f:
        f.write(l)

    print("\tSaved image to data folder.")
    print("\tProcessing image for display.")

    f.close()


    g = b''
#        for i in range(1,len(l[l[2]+7:]),2):
    for i in range(0,len(l)-1,2):
        g += bytes([l[i]])

    print("\tDisplaying image.")
    print(len(g))
    g = g[:76800]
    if len(g) <= 320*240:
        g += bytes(320*240-len(g))
    print("g is " + str(len(g)) + " bytes")
    im = Image.frombytes("L", (320,240), g)
    im.show()
    im.save(pngname)


if __name__ == '__main__':
    main()
