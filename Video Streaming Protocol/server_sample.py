import socket
import sys
import os

HOST = '10.120.54.48' #All available interfaces
PORT = 8888 #Arbitrary non-priviledged port

#create socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
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
    conn, addr = s.accept()
    print 'Connected with ' + addr[0] + ':' + str(addr[1])
    #fork for each connection
    if os.fork()==0:
        while 1:
            data = conn.recv(1024)
            reply = 'OK...' + data
            if not data:
                conn.close()
                sys.exit()
            parse = data.split()
            if parse[0] == "TEST":
                conn.sendall("testing")
            conn.sendall(reply)

conn.close()
s.close()
