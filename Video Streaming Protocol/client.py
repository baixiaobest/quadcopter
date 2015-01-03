import socket
import sys
import cv2
import select
import numpy as np


def connect(ip, port):
    #create socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    except socket.error, msg:
        print "Fail to create socket"
        sys.exit()
    s.connect((ip, port))
    return s

def queryFrame(conn, count):
    if count == 1:
        conn.sendall("FRAME")
        if select.select([conn], [], [], 10)[0]:
            format = conn.recv(1024)
            parse = format.split()
            if parse[0]=="IMGFOR":
                if parse[1] == "RAW":
                    height = int(parse[2])
                    width = int(parse[3])
                    channel = int(parse[4])
                    conn.sendall("OK")
                    if channel == 1:
                        if select.select([conn], [], [], 3)[0]:
                            data = [ord(c) for c in conn.recv(height*width)]
                            numpyData = np.uint8(data[:])
                            conn.sendall("OK")
                            return numpyData.reshape(height,width)
                    elif channel == 3:
                        numpyData = np.zeros((channel, width*height), dtype=np.uint8)
                        for i in range(0,3):
                            if select.select([conn], [], [], 3)[0]:
                                data = [ord(c) for c in conn.recv(height*width)]
                                numpyData[i,:] = np.uint8(data[:])
                                conn.sendall("OK")
                            else:
                                return None
                        img = cv2.merge((numpyData[0,:].reshape(height,width), numpyData[1,:].reshape(height,width),numpyData[2,:].reshape(height,width)))
                        return img
                if parse[1]=="JPG":
                    return None
    return None


if __name__=="__main__":
    conn = connect("10.120.54.48",8888)
    while 1:
        parse = None
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            stdinput = sys.stdin.readline()[:-1]
            parse = stdinput.split()
            if parse[0] == "FRAME":
                while 1:
                    print "Query frame"
                    frame = queryFrame(conn, 1)
                    if frame is not None and frame.size>0:
                        cv2.imshow("test",frame)
                        cv2.destroyAllWindows()
                    c = cv2.waitKey(1)
                    if c == 27:
                        sys.exit()
            if parse[0] == "OK":
                conn.sendall("OK")

