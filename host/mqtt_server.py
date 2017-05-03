import paho.mqtt.client as mqtt
import fifo
import time
import select
from PIL import Image
import threading

serial_list = b''
serial_count = 0
serial_fifo = fifo.BytesFIFO(640*480*2)
start = 0
end = 0
threads = []

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
    client.subscribe('img/data')
    client.subscribe('img/start')
    client.subscribe('img/end')

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global serial_list
    global serial_fifo
    global serial_count
    global start
    global end
    if(msg.topic == 'img/start'):
        start = time.time()
        print('Start image recieved')
        print(msg.payload)
        serial_list = b''
        serial_count = 0
        serial_fifo.read(len(serial_fifo))
        client.publish('img/start/ack', 'yee', 0, 0)
    elif(msg.topic == 'img/end'):
        end = time.time()
        upload = end - start
        print('Upload took ' + str(upload) + ' seconds')
        print('Number of bytes recieved: ' + str(len(serial_fifo)))
        print(time.strftime("%Y%m%d_%H%M%S", time.gmtime()))
        t = threading.Thread(target=display_image, args=(serial_fifo.read(len(serial_fifo)),))
        threads.append(t)
        t.start()
        #display_image(serial_fifo.read(len(serial_fifo)))
        #client.publish('img/end/ack', 'data', 1, 0)
    elif(msg.topic == 'img/data'):
        serial_count += len(msg.payload)
        serial_fifo.write(msg.payload)
        client.publish('img/data/ack', 'end', 0, 0)

def ycbcr2rgb(y,cb,cr):

    #R = int(298.082*y/256 + 408.583*cr/256 - 222.912)
    R = int(1.64*(y-16) + 1.596*(cr-128))
    if R > 255:
        R = 255
    if R < 0:
        R = 0
    #G = int(298.082*y/256 + 100.291*cb/256 - 208.120*cr/256 + 135.576)
    G = int(1.64*(y-16) - 0.813*(cr-128) - 0.391*(cb-128))
    if G > 255:
        G = 255
    if G < 0:
        G = 0

    #B = int(298.082*y/256 + 516.412*cb/256 - 276.836)
    B = int(1.64*(y-16) + 2.018*(cb-128))
    if B > 255:
        B = 255
    if B < 0:
        B = 0

    return bytes([R,G,B])

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
    for i in range(0,len(l)-1,4):
        g += ycbcr2rgb(l[i], l[i+1], l[i+3])
        g += ycbcr2rgb(l[i+2], l[i+1], l[i+3])
#        g += bytes([l[i]])

    print("\tDisplaying image.")
    print(len(g))
    if len(g) <= 640*480:#320*240:
        g += bytes(640*480-len(g))#(320*240-len(g))
    print("g is " + str(len(g)) + " bytes")
    im = Image.frombytes("RGB", (640,480), g)
    im.show()
    im.save(pngname)


if __name__ == '__main__':
    main()
