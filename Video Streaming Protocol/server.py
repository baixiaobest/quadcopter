import socket
import sys
import os
import cv2
import numpy as np
import select
import pickle
from thread import *
camera = 0

def processRequest():
    if select.select([conn], [], [], 5)[0]:
        data = conn.recv(1024)
        if not data:
            return
        parse = data.split()
        if len(parse) != 0:
            if parse[0] == "TEST":
                conn.sendall("testing")
            if parse[0] == "EXIT":
                    sys.exit()
            if parse[0] == "FRAME":
                format = "RAW"
                if len(parse) > 1:
                    format = parse[1]
                frameQuery(conn, format)
                    
def frameQuery(conn, format):
    global camera
    if not camera:
        camera = cv2.VideoCapture(0)
    ret, colorframe = camera.read()
    #frame = cv2.cvtColor(cv2.resize(colorframe, (60,60)), cv2.COLOR_BGR2GRAY)
    frame = cv2.resize(colorframe, (100,100))
    #frame = colorframe
    try:
        height, width, channel = frame.shape
    except:
        channel = 1
        height, width = frame.shape
    if format == "RAW":
        conn.sendall("IMGFOR RAW {0} {1} {2}".format(height, width, channel))
    while 1:
        if select.select([conn], [], [], 0)[0]:
            data = conn.recv(1024)
            parse = data.split()
            if len(parse)!=0 and parse[0] == "OK":
                break
    print "start sending image"
    conn.setblocking(0)
    #cv2.imshow("server", frame)
    img = [[0 for x in range(height*width)] for y in range(channel)]
    if channel == 3:
        for i in range(0, height):
            img[0][i*width:(i+1)*width] = frame[i,:,0]
            img[1][i*width:(i+1)*width] = frame[i,:,1]
            img[2][i*width:(i+1)*width] = frame[i,:,2]
    else:
        for i in range(0, height):
            img[0][i*width:(i+1)*width] = frame[i,:]

    for i in range(0,channel):
        print i
        data = [chr(c) for c in img[i][:]]
        dataStream = ''.join(data)
        conn.sendall(dataStream)
        try:
            if select.select([conn], [], [], 10)[0]:
                if conn.recv(1024).split()[0]!="OK":
                    break
            else: break
        except:
            break
    print "end"



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