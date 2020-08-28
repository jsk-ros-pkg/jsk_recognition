#!/usr/bin/env python

import cv2
import sys
import os

def waitKeyInput():
    while True:
        k = cv2.waitKey(33)
        if k == 110:                      #n
            return False
        elif k == 121:                    #y
            return True

def main():
    info = sys.argv[1]
    output = sys.argv[2]
    if not os.path.exists(info):
        raise Exception("failed to find %s" % info)
    with open(info, "r") as input_f:
        lines = input_f.readlines()
        cv2.namedWindow("image")
        results = []
        for line in lines:
            try:
                (fname, num, x, y, width, height) = [c for c in line.split() 
                                                     if c and c.strip("\n ")]
                x = int(x)
                y = int(y)
                width = int(width)
                height = int(height)
                if os.path.exists(fname):
                    print("Opening " + fname)
                    img = cv2.imread(fname)
                    cv2.rectangle(img, (x, y), 
                                  (x + width, y + height),
                                  (0, 0, 255), 3)
                    cv2.imshow("image", img)
                    usep = waitKeyInput()
                    if usep:
                        results.append((fname, num, x, y, width, height))
            except:
                 pass
        with open(output, "w") as output_f:
             for result in results:
                 output_f.write("%s %d %d %d %d %d\n" % (result[0],
                                                         1,
                                                         result[2],
                                                         result[3],
                                                         result[4],
                                                         result[5]))
            
if __name__ == "__main__":
    main()
    
