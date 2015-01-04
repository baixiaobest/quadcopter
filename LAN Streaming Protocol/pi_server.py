import socket
import sys
import os
import cv2
import numpy as np
import select
import picamera
from thread import *

camera = 0
stream = io.BytesIO()
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

def processRequest():
    while 1:
        if select.select([conn], [], [], 5)[0]:
            data = conn.recv(1024)
            if not data:
                return
            parse = data.split()
            #server request
            if len(parse) != 0:
                if parse[0] == "TEST":
                    conn.sendall("testing")
                if parse[0] == "EXIT":
                    sys.exit()
                if parse[0] == "FRAME":
                    format = "JPG"
                    if len(parse) > 1:
                        format = parse[1]
                    frameQuery(conn, format)

def frameQuery(conn, format):
    if camera == 0:
        with picamera.PiCamera() as camera:
            camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT)
    camera.capture(stream, format='jpeg')
    numpyData = np.fromstring(stream.getvalue(), dtype=np.uint8)
    frame = cv2.imdecode(data, cv2.CV_LOAD_IMAGE_COLOR)
    try:
        height, width, channel = frame.shape
    except:
        channel = 1
        height, width = frame.shape
    
    #JPG format
    if format == "JPG":
        retval, compressed = cv2.imencode('.jpg', frame)
        rows, cols = compressed.shape
        conn.sendall("IMGFOR JPG {0} {1}".format(rows, cols))
    if select.select([conn], [], [], 1)[0]:
        data = conn.recv(1024)
        parse = data.split()
        if len(parse)==0 or parse[0] != "OK":
            return
    #marshal the compressed data to string
    conn.sendall(''.join([chr(c) for c in compressed[:,0]]))

if __name__=="__main__":
    HOST = '10.120.54.48' #All available interfaces
    PORT = 8888#Arbitrary non-priviledged port
    
    #create socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setblocking(1)
    print 'Socket Created'
    
    #bind to port
    try:
        s.bind((HOST, PORT))
    except socket.error, msg:
        print 'Bind Failed'
        sys.exit()
    print 'Socket bind complete'

#listen to socket
s.listen(10)
    print 'Socket is listening'
    
    #accept incoming request
    while 1:
        #wait to accept a connection - blocking call
        if select.select([s], [], [], 0)[0]:
            conn, addr = s.accept()
            print 'Connected with ' + addr[0] + ':' + str(addr[1])
            #fork for each connection
            #if os.fork()==0:
            print "processing request"
            processRequest()
conn.close()
    s.close()