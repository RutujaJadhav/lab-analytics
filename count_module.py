from __future__ import print_function
from imutils.object_detection import non_max_suppression
import numpy as np
import argparse
import imutils
import cv2
import paho.mqtt.client as mqtt
import time
from matplotlib import pyplot as plt
fig=plt.figure()
plt.axis([0,1000,0,1])

i=0
x=list()
y=list()


broker="iot.eclipse.org"
port=1883

DEFAULT_MIN_IMAGE_WIDTH = 400
DEFAULT_IMAGE_PADDING = (16, 16)
SHOW_ORIGINAL_IMG = True

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

cap = cv2.VideoCapture("rtsp://173.39.91.104/StreamingSetting?version=1.0&action=getRTSPStream&sessionID=41454973&ChannelID=1&ChannelName=Channel1")
#cap.set(CV_CAP_PROP_FPS, 10);
#fps = cap.get(cv2.CV_CAP_PROP_FPS)
#print ("Frames per second using video.get(cv2.cv.CV_CAP_PROP_FPS): {0}".format(fps))
def on_log(client, userdata, level, buf):
        print (buf)
def on_connect(client,userdata,flags,rc):
        if rc==0:
                client.connected_flag=True
                print ("connected OK")
        else:
                print ("Bad connection retured code=",rc)
                client.loop_stop()
def on_disconnect(client,userdata,rc):
        print ("client disconnected OK")
def on_publish(client,userdata, mid):
        print("in on_pub callback mid=", mid)

mqtt.Client.connected_flag=False
client = mqtt.Client("python1")
client.on_log=on_log
client.on_connect=on_connect
client.on_disconnect=on_disconnect
client.on_publish=on_publish
client.connect(broker,port)
client.loop_start()
while(1):
        t = time.time()
        ret, image = cap.read()
        (rects, wghts) = hog.detectMultiScale(image,winStride=(5,5),padding=(16,16), scale=1.05)

        def __init__(self):
        # default mode
                self.MIN_IMAGE_WIDTH, self.WIN_STRIDE, self.PADDING, self.SCALE, self.OVERLAP_THRESHOLD = self.CALIBRATION_MODE_5
                self.SHOW_IMAGES = True
                self.IMAGE_WAIT_TIME = 0  # Wait indefinitely until button pressed
                self.GRAY_SCALE = False

        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.9)

#        for (xA, yA, xB, yB) in pick:

        print("{} people detected.\n".format(len(pick)))

        while not client.connected_flag:
            print ("In wait loop")
            time.sleep(1)
        print ("publishing")
        ret=client.publish("qbator/people",len(pick),0)
        print ("publishing return",ret)



        if SHOW_ORIGINAL_IMG:
                cv2.imshow("After Detection", image)
                k = cv2.waitKey(30) & 0xff
                if k == 27:
                        break
    
        x.append(t)
        y.append(len(pick))
        plt.scatter(t,len(pick))
        i+=1
        plt.show()

cap.release()
cv2.destroyAllWindows()


#qbator/control --- kitni light jalani hai
#qbator/call_temp
#qbator/auto ---0 and 1



