import socket
import sys
import cv2
import select
import numpy as np
import time


def connect(ip, port):
    #create socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    except socket.error, msg:
        print "Fail to create socket"
        sys.exit()
    s.connect((ip, port))
    return s

def queryFrame(conn):
    conn.sendall("FRAME")
    if select.select([conn], [], [], 10)[0]:
        format = conn.recv(1024)
        parse = format.split()
        if parse[0]=="IMGFOR":
            if parse[1]=="JPG":
                size = int(parse[2])
                conn.sendall("OK")
                if select.select([conn], [], [], 0.1)[0]:
                    data = [[ord(c)] for c in recvAll(conn, size)]
                    compressed = np.uint8(data)
                    actualsize = compressed.size
                    #print "expected {0} actual {1}".format(size,actualsize)
                    image = cv2.imdecode(compressed, cv2.CV_LOAD_IMAGE_COLOR)
                    return image
                else:
                    return None
    return None

def recvAll(conn, size):
    dataString = ""
    while 1:
        if len(dataString) == size:
            return dataString
        if select.select([conn],[],[],0.1):
            dataString += conn.recv(size)
        #if no incoming data within a sec, drop it
        else:
            return None


if __name__=="__main__":
    conn = connect("10.120.54.48",8888)
    print "server connected"
    while 1:
        parse = None
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            stdinput = sys.stdin.readline()[:-1]
            parse = stdinput.split()
            if parse[0] == "FRAME":
                count = 0
                while 1:
                    frame = queryFrame(conn)
                    if frame is not None and frame.size>0:
                        cv2.imshow("test".format(count),frame)
                    else:
                        print "Drop Frame"
                    c = cv2.waitKey(1)
                    if c == 27:
                        sys.exit()
                    count += 1
            if parse[0] == "OK":
                conn.sendall("OK")

