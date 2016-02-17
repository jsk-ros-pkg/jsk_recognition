#!/usr/bin/env python

import matplotlib.pyplot as plt
import sys
import csv

xs = []
ys = []
for row in csv.reader(open(sys.argv[1], 'r')):
    if row:
        xs.append(row[3])
        ys.append(row[1])
plt.plot(xs, ys)
plt.xlabel("number of points")
plt.ylabel("Time to estimate plane [sec]")
plt.interactive(True)
plt.show()
    
while True:
    plt.pause(1)
