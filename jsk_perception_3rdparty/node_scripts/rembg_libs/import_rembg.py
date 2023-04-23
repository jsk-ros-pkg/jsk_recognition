import imp
import os.path as osp


abs_path = osp.dirname(osp.abspath(__file__))


rembg = imp.load_package(
    'rembg', osp.join(abs_path, 'rembg/rembg'))
