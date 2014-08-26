#coding:utf-8
import numpy, pylab

# http://aidiary.hatenablog.com/entry/20100404/1270359720

class BayesianCurveFitting(object):
    def __init__(self, M, ALPHA, BETA, xlist, tlist):
        self.M = M
        self.ALPHA = ALPHA
        self.BETA = BETA
        self.xlist = xlist
        self.tlist = tlist

    def y(self, x, wlist):
        ret = wlist[0]
        for i in range(1, self.M + 1):
            ret += wlist[i] * (x ** i)
        return ret

    def phi(self, x):
        data = []
        for i in range(0, self.M + 1):
            data.append(x**i)
        ret = numpy.matrix(data).reshape((self.M + 1, 1))
        return ret

    # eq (1.70)
    def mean(self, x, S):
        sums = pylab.matrix(pylab.zeros((self.M + 1, 1)))
        for n in range(len(self.xlist)):
            sums += self.phi(self.xlist[n]) * self.tlist[n]
        ret = self.BETA * self.phi(x).transpose() * S * sums
        return ret

    # eq (1.71)
    def variance(self, x, S):
        ret = 1.0 / self.BETA + self.phi(x).transpose() * S * self.phi(x)
        return ret

    def main(self):
        sums = pylab.matrix(pylab.zeros((self.M + 1, self.M + 1)))
        for n in range(len(self.xlist)):
            sums += self.phi(self.xlist[n]) * self.phi(self.xlist[n]).transpose()
        I = pylab.matrix(numpy.identity(self.M + 1))
        S_inv = self.ALPHA * I + self.BETA * sums
        S = S_inv.getI()

        xs = numpy.linspace(min(self.xlist), max(self.xlist), 500)
        means = []
        uppers = []
        lowers = []
        for x in xs:
            m = self.mean(x, S)[0, 0]
            s = numpy.sqrt(self.variance(x, S)[0, 0])
            u = m + s
            l = m - s
            means.append(m)
            uppers.append(u)
            lowers.append(l)
        return([xs, means, uppers, lowers])
