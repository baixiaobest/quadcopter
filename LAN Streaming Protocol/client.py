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
                if select.select([conn], [], [], 1)[0]:
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
        if select.select([conn],[],[],1):
            dataString += conn.recv(size)
        #if no incoming data within a sec, drop it
        else:
            return None


if __name__=="__main__":
    try:
        conn = connect("169.232.187.81",8888)
    except:
        conn = connect("169.232.187.81",8887)
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
                        cv2.imshow("test",frame)
                    else:
                        print "Drop Frame"
                        while select.select([conn], [], [], 1)[0]:
                            conn.recv(1024)
                    c = cv2.waitKey(1)
                    if c == 27:
                        sys.exit()
                    count += 1
            #frame subscription
            if parse[0] == "SUBFRAME":
                numFrames = 100
                if len(parse) == 2:
                    numFrames = parse[1]
                conn.sendall("SUBFRAME {0}".format(numFrames))
                if select.select([conn],[],[],1)[0]:
                    format = conn.recv(128)
                    parse = format.split()
                    if len(parse)<=2 and parse[0]!="IMGFOR" and parse[1]!="JPG":
                        continue
                    conn.sendall("OK")
            

                dataArray = []
                while 1:
                    dataString = ""
                    endOfData = False
                    if select.select([conn], [], [], 1)[0]:
                        dataString = conn.recv(60000)
                        if "end" in dataString:
                            dataString = dataString[:-3]
                            endOfData = True
                        if "start" in dataString:
                            dataArray[len(dataArray):] = filter(None,dataString.split("start"))
                    if endOfData:
                        break
                for rawData in dataArray:
                    compressed = np.uint8([[ord(c)] for c in rawData])
                    image = cv2.imdecode(compressed, cv2.CV_LOAD_IMAGE_COLOR)
                    if image is not None and image.size != 0:
                        cv2.imshow("test",image)
                    c = cv2.waitKey(10)

                cv2.destroyAllWindows()
                        
                
            if parse[0] == "OK":
                conn.sendall("OK")

