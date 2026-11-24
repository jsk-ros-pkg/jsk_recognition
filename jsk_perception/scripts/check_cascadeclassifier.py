#!/usr/bin/env python

import numpy as np
import cv2
import sys

def main():
    if len(sys.argv) < 3:
        print("Usage: check_cascadeclassifier.py cascade.xml [image_files ...]")
        sys.exit(1)
    classifier_file = sys.argv[1]
    files = sys.argv[2:]
    cascade = cv2.CascadeClassifier(classifier_file)
    for f in files:
        img = cv2.imread(f)
        try:
            if img != None:
                faces = cascade.detectMultiScale(img, 1.3, 10)
                for (x,y,w,h) in faces:
                    cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
                cv2.imshow('img', img)
                cv2.waitKey(100)
        except KeyboardInterrupt as e:
            raise e
        except Exception as e:
            pass

if __name__ == "__main__":
    main()
    
