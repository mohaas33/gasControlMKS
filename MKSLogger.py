#! /usr/bin/python3

from readMKS import readMKS

import re 

import os

import sys
import time


def main():
    flow_4 = re.sub("[^0-9+-]", "", readMKS('FL 4'))
    flow_6 = re.sub("[^0-9+-]", "", readMKS('FL 6'))
    pressure = re.sub("[^0-9+-]", "", readMKS('PR'))
    s = flow_4 + ' ' + flow_6 + ' ' + pressure
    #print(s)
    return s
    
if __name__ == "__main__":
  args = sys.argv[1:]
  filename = args[0]
  append_write = ''
  if os.path.exists(filename):
    append_write = 'a' # append if already exists
  else:
    append_write = 'w' # make a new file if not  

  # open the file
  outF = open(filename, append_write )
  time_int = int(args[1])
  dt = int(args[2])
  for t in range(time_int):
    # write line to output file
    s = main()
    outF.write(s)
    outF.write("\n")
    time.sleep(dt)
  outF.close()
   