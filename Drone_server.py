#!/usr/bin/python
# coding: utf-8
import socket
import struct

s = socket.socket()
host = ''
port = 12345
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((host, port))
s.listen(5)

print "I am waiting"
i = 0
f = 0
while True:
    c, addr = s.accept()
    data = c.recv(4096)
    Data = struct.unpack('>ff',data)
    print Data
    c.close()

