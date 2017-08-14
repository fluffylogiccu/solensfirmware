import paho.mqtt.client as mqtt
import fifo
import time
import select
from PIL import Image
import threading
import os
import errno
import string

#serial_list = b''
serial_count = 0


#List of Serial FIFO objects
serial_fifos = []
#List of IDs
ids = []
#serial_fifo = fifo.BytesFIFO(640*480*8)
start = 0
end = 0
threads = []

upload = 0


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
    client.subscribe('img/start') #Only subscribe to img/start to being with

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    #global serial_list
    #global serial_fifo
    global serial_count
    global start
    global end
    global upload
    global ids
    global serial_fifos
    #print(msg.topic);
    if(msg.topic == 'img/start'):
        #Send signal back to board that we've received the code
        #client.publish('img/start/ack', 'yee', 0, 0)

        start = time.time()
        print('Start image received')
        SNid = msg.payload


        #Parse out the ID
        strSNid= str(SNid)
        strSNid= strSNid[2:-1]

        if strSNid in ids: #Check if id is already in list, if so, empty corresponding FIFO
            ind = ids.index(strSNid)
            serial_fifo = serial_fifos[ind]
            serial_fifo.empty()
        else:
            #Add to ids list
            ids.append(strSNid)

            #Create new serial fifo and add to fifos list
            serial_fifo = fifo.BytesFIFO(640*480*4)
            serial_fifo.read(len(serial_fifo))
            serial_fifos.append(serial_fifo)

        #### At this point, the id and the serial_fifos will have the same index in both lists, thus linking them


        print("Subscribing to: " + "img/data/" + strSNid) #Allows for server to dynamically subscribe to individual esp-links
        print("Subscribing to: " + "img/end/" + strSNid)  
        client.subscribe('img/data/' + strSNid)
        client.subscribe('img/end/' + strSNid)
        #serial_list = b''
        serial_count = 0

    elif(msg.topic[:10] == 'img/end/SN'):
        end = time.time()
        #Full topic = img/end/SN000000001 so id is last part
        SerialID = msg.topic[8:]
        #Find index of id
        ind = ids.index(SerialID);
        #index of id is same for corresponding serial_fifo bc they were added together
        serial_fifo = serial_fifos[ind]
        upload = end - start
        print('Upload took ' + str(upload) + ' seconds')
        print('Number of bytes received: ' + str(len(serial_fifo)))
        print(time.strftime("%Y%m%d_%H%M%S", time.gmtime()))
        client.unsubscribe('img/data/' + SerialID)
        client.unsubscribe('img/end/' + SerialID)
        t = threading.Thread(target=display_image, args=(serial_fifo.read(len(serial_fifo)),ind))
        threads.append(t)
        t.start()
        #display_image(serial_fifo.read(len(serial_fifo)))
        #client.publish('img/end/ack', 'data', 1, 0)
    elif(msg.topic[:11] == 'img/data/SN'):
        #client.publish('img/data/ack', 'end', 0, 0)
        #Same procedure as img/end
        SerialID = msg.topic[9:] #Add this back later
        #SerialID = "SN000000001" #Temporary hard code
        SNid= SerialID
        ind = ids.index(SerialID);
        serial_fifo = serial_fifos[ind]

        serial_count += len(msg.payload)
        serial_fifo.write(msg.payload)
        print("Received")

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

def display_image(l,ind):
    print(len(l))
    global upload
    global serial_fifos
    global ids

    # JK -- Use index to get ID from dynamically allocated ID list
    strSNid = ids[ind]
    

    # RF --  Need to match ID to static list of IDs
    # ex: key: value ---> ID : LatLong
    # SN 0000 00001
    latlongidpairs= {'SN000000001': '40N20W', 'SN00000002': '40N30W'}
    LatLong= latlongidpairs[strSNid]
    # print('ID: ' + strSNid + ' LatLong: ' + LatLong)

    del ids[ind]
    del serial_fifos[ind]


    # save stuff in ~/data/ directory
    csvname = 'data/timings.csv' # Used to save the upload times for analysis

    # create filenames
    filename = 'data/' + strSNid + '/' + strSNid + '_' + time.strftime("%Y%m%d_%H%M%S", time.gmtime()) + '_' + LatLong + '.raw'
    pngname =  'data/' + strSNid + '/' + strSNid + '_' + time.strftime("%Y%m%d_%H%M%S", time.gmtime()) + '_' + LatLong + '.png' 
    jpgname =  'data/' + strSNid + '/' + strSNid + '_' + time.strftime("%Y%m%d_%H%M%S", time.gmtime()) + '_' + LatLong + '.jpg'

    if not os.path.exists(os.path.dirname(jpgname)):
        try:
            os.makedirs(os.path.dirname(jpgname))
        except OSError as exc: # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise


    with open (csvname, "a") as csvFile: #Append the upload times
        csvFile.write(str(upload) + ',')

   
    #Code Added by Justin to support JPG
    with open(jpgname, "wb") as jpgFile:
        jpgFile.write(l)

    jpgFile.close();
    ####


######################## Code to deal with non-JPEG images


    if not os.path.exists(os.path.dirname(filename)):
        try:
            os.makedirs(os.path.dirname(filename))
        except OSError as exc: # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise

    if not os.path.exists(os.path.dirname(pngname)):
        try:
            os.makedirs(os.path.dirname(pngname))
        except OSError as exc: # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise

    print("\tSaved image to data folder.")
    print("\tProcessing image for display.")


    with open(filename, "wb") as f:
        f.write(l)
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

# ######################

    # ################## Code added by Justin to support JPG
    # print ("Displaying Image");
    # im2 = Image.open(jpgname);
    # im2.show();
    # im2.save(jpgname);
    # ####################



if __name__ == '__main__':
    main()
