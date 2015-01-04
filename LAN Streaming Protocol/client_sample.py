import socket
import sys

#create socket
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
except socket.error, msg:
    print "Fail to create socket"
    sys.exit();

print 'Socket Created'

host = 'www.google.com'
port = 80

#grab ip of server
try:
    remote_ip = socket.gethostbyname(host)
except socket.gaierror:
    print 'Hostname not resolved'
    sys.exit()

print 'IP addr of ' + host + ' is ' + remote_ip
#connect to server
s.connect((remote_ip, port))
print 'Socket connected to ' + host + ' on IP ' + remote_ip

#send message
message = "GET / HTTP/1.1\r\n\r\n"
try:
    s.sendall(message)
except socket.error:
    print 'Send Failed'
    sys.exit()
print 'Message sent successfully'

#receive data
reply = s.recv(4096)
print reply

s.close()