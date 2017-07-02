#!/usr/bin/python
# coding=utf-8

# Source:
# https://stackoverflow.com/questions/22397289/finding-the-values-of-the-arrow-keys-in-python-why-are-they-triples
import sys,tty,termios
class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

def get():
    inkey = _Getch()
    while(1):
            k=inkey()
            if k!='':break
    print 'you pressed', ord(k)

def main():
    for i in range(0,25):
        get()

if __name__=='__main__':
    main()
keycode
