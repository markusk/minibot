#!/usr/bin/env python
# coding=utf-8


# for getting the hostname and IP of the underlying system
import socket
import subprocess

""" import fcntl
import struct """

# get hostname
hostname = socket.gethostname()

# get IP
ip = subprocess.check_output(['hostname', '-I'])
# erstes leerzeichen finden und dort abschneiden
ip4 = ip[:ip.index(" ")]

# show it
print("Running on host %s.", hostname)
print("IP4 address is %s.", ip4)
