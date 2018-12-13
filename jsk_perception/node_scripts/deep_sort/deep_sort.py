import imp
import os.path as osp


abs_path = osp.dirname(osp.abspath(__file__))


deep_sort = imp.load_package(
    'deep_sort', osp.join(abs_path, 'deep_sort/deep_sort'))

application_util = imp.load_package(
    'application_util', osp.join(abs_path, 'deep_sort/application_util'))

deep_sort_app = imp.load_source(
    'deep_sort_app', osp.join(abs_path, 'deep_sort/deep_sort_app.py'))
