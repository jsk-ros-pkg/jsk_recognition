#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import matplotlib.mlab as mlab
import math
import sys

if len(sys.argv) != 3:
    print("plot_gaussian.py mean variance")
    sys.exit(1)
mean = float(sys.argv[1])
variance = float(sys.argv[2])
sigma = math.sqrt(variance)
x = np.linspace(-3*sigma+mean,3*sigma+mean,100)
plt.plot(x,mlab.normpdf(x,mean,sigma))

plt.show()
