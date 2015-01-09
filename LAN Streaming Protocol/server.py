import socket
import sys
import os
import cv2
import numpy as np
import select
from thread import *
camera = 0

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
                #request one frame
                if parse[0] == "FRAME":
                    format = "JPG"
                    if len(parse) > 1:
                        format = parse[1]
                    frameQuery(conn, format)
                #subscribe for frames
                if len(parse) == 2 and parse[0] == "SUBFRAME":
                    numFrames = int(parse[1])
                    subscribeFrames(conn, numFrames)

def frameQuery(conn, format):
    ret, frame = captureFrame()
    frame = cv2.resize(frame, (640,480))
    height, width, channel = frame.shape

    #JPG format
    if format == "JPG":
        retval, compressed = cv2.imencode('.jpg', frame)
        rows, cols = compressed.shape
        conn.sendall("IMGFOR JPG {0} {1}".format(rows, cols))
    if select.select([conn], [], [], 0.1)[0]:
        data = conn.recv(1024)
        parse = data.split()
        if len(parse)==0 or parse[0] != "OK":
            return
        #marshal the compressed data to string
        conn.sendall(''.join([chr(c) for c in compressed[:,0]]))

def subscribeFrames(conn, numFrames):
    #motion JPEG
    conn.sendall("IMGFOR JPG")
    if select.select([conn],[],[],1)[0]:
        data = conn.recv(1024)
        parse = data.split()
        if len(parse)==0 or parse[0] != "OK":
            return

        for i in range (0,numFrames):
            ret, frame = captureFrame()
            frame = cv2.resize(frame, (640, 480))
            retval, compressed = cv2.imencode('.jpg', frame)
            size, cols = compressed.shape
            sendData = "start"+''.join([chr(c) for c in compressed[:,0]])
            conn.sendall(sendData)
        
        conn.sendall("end")

def marshalImage(img):
    _, compressed = cv2.imencode('.jpg',img)
    return ''.join([chr(c) for c in compressed[:,0]])

def captureFrame():
    global camera
    if not camera:
        camera = cv2.VideoCapture(0)
    return camera.read()

if __name__=="__main__":
    HOST = str(socket.gethostbyname(socket.gethostname())) #All available interfaces
    PORT = 8888#Arbitrary non-priviledged port
    
    #create socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setblocking(1)
    print 'Socket Created'
    
    #bind to port
    try:
        s.bind((HOST, PORT))
    except socket.error, msg:
        PORT = 8887
        s.bind((HOST, PORT))
        #print 'Bind Failed'
        #sys.exit()
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