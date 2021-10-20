#! /usr/bin/python3

from readMKS import readMKS

import sys

def main(argv):
    command = ' '.join(argv)
    e = readMKS(command)
    print(e)
    return e
    
if __name__ == "__main__":
   main(sys.argv[1:])