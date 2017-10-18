#!/usr/bin/env python
# coding=utf-8


# for getting the hostname and IP of the underlying system
import socket
import subprocess


# get hostname
hostname = socket.gethostname()

# get IP via shell
ip = subprocess.check_output(['hostname', '-I'])
# find first space and cut string at this index
ip4 = ip[:ip.index(" ")]

# show it
print("Running on host %s.", hostname)
print("IP4 address is %s.", ip4)
