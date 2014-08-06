import rosbag
import argparse
import matplotlib.pyplot, numpy, mpl_toolkits.mplot3d.axes3d
from BayesianCurveFitting import *
from math import *

def parse_args():
    parser = argparse.ArgumentParser(description='plot data from rosbag')
    parser.add_argument('inbag', type=str, help='input bag file', metavar='bagfile')
    parser.add_argument('-t', type=int, help='1 or 2', metavar='graph type', choices=[1, 2], default=2)
    parser.add_argument('-u', type=int, help='step in u', default=80)
    parser.add_argument('-v', type=int, help='step in v', default=80)
    parser.add_argument('-d', type=float, help='step in depth', default=0.1)
    parser.add_argument('--max_depth', type=float, help='maximum depth', default=1.5)
    parser.add_argument('-D', type=int, help='D degree polynomial', default=2)
    parser.add_argument('-a', type=float, help='ALPHA', default=0.005)
    parser.add_argument('-b', type=float, help='BETA', default=11.1)
    return parser.parse_args()

def setup_data_list(v_num, u_num):
    data_list = []
    for v_i in range(0, v_num):
        data_list.append([])
        for u_i in range(0, u_num):
            data_list[v_i].append([])
    return data_list

def process(args):
    width = 640
    height = 480
    step_u = args.u
    step_v = args.v
    data_list = setup_data_list(height/step_v, width/step_u)
    f, (axes_array) = matplotlib.pyplot.subplots(len(data_list), len(data_list[0]), sharex='col', sharey='row')
    with rosbag.Bag(args.inbag) as b:
        for topic, msg, t in b.read_messages():
            u_i = msg.u / step_u
            v_i = msg.v / step_v
            if abs((msg.observed_depth - msg.true_depth) * 1000) < 1000: # for outlier
                data_list[v_i][u_i].append([msg.observed_depth, # x, [m]
                                            (msg.observed_depth - msg.true_depth) * 1000]) # y, [mm]
    for x in range(0, len(axes_array[0])):
        for y in range(0, len(axes_array)):
            xlist = [tmp[0] for tmp in data_list[y][x]]
            ylist = [tmp[1] for tmp in data_list[y][x]]
            bcf = BayesianCurveFitting(args.D, args.a, args.b, xlist, ylist).main()
            axes_array[y][x].plot(xlist, ylist, '.')
            axes_array[y][x].plot(bcf[0], bcf[1], 'r-')
            axes_array[y][x].plot(bcf[0], bcf[2], 'g--')
            axes_array[y][x].plot(bcf[0], bcf[3], 'g--')
            axes_array[y][x].set_title('u : %d~%d, v : %d~%d' % (x*step_u, (x+1)*step_u, y*step_v, (y+1)*step_v))
    matplotlib.pyplot.show()


def setup_data_list2(d_num):
    data_list = []
    for d_i in range(d_num):
        data_list.append([])
    return data_list

def process2(args):
    max_depth = args.max_depth
    step_d = args.d
    data_list = setup_data_list2(int(max_depth / step_d))
    fig = matplotlib.pyplot.figure(figsize=matplotlib.pyplot.figaspect(0.5))
    axes_array = []
    for i in range(len(data_list)):
        axes_array.append(fig.add_subplot(int(sqrt(len(data_list))) + 1, int(sqrt(len(data_list))) + 1, i+1, projection='3d'))
    with rosbag.Bag(args.inbag) as b:
        for topic, msg, t in b.read_messages():
            if msg.true_depth < step_d * len(data_list):
                d_i = int(msg.true_depth / step_d)
                if abs((msg.observed_depth - msg.true_depth) * 1000) < 1000: # for outlier
                    data_list[d_i].append([msg.u, msg.v,
                                           (msg.observed_depth - msg.true_depth) * 1000])
    for d in range(len(axes_array)):
        xlist = numpy.array([tmp[0] for tmp in data_list[d]])
        ylist = numpy.array([tmp[1] for tmp in data_list[d]])
        X, Y = numpy.meshgrid(xlist, ylist)
        zlist = numpy.array([tmp[2] for tmp in data_list[d]])
        axes_array[d].scatter3D(xlist, ylist, zlist, c='b', marker='o')
        # axes_array[d].plot_surface(X, Y, zlist)
        # axes_array[d].plot_wireframe(xlist, ylist, zlist)
        axes_array[d].set_title('d : %.2f~%.2f[m]' % (d*step_d, (d+1)*step_d))
    matplotlib.pyplot.show()



def main():
    args = parse_args()
    if args.t == 1:
        process(args)
    else:
        process2(args)

if __name__ == '__main__':
    main()
