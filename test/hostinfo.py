#!/usr/bin/env python
# coding=utf-8


# for getting the hostname and IP of the underlying system
import socket
import fcntl
import struct

# get hostname
hostname = socket.gethostname()

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # # 0x8915 is SIOCGIFADDR
    return socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', ifname[:15]))[20:24])

# show it
print("Running on host %s.", hostname)
print("IP address of eth0 is %s.", get_ip_address('eth0'))
