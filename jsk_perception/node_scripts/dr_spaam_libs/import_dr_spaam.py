import imp
import os.path as osp


abs_path = osp.dirname(osp.abspath(__file__))


dr_spaam = imp.load_package(
    'dr_spaam', osp.join(abs_path, 'dr_spaam/dr_spaam/dr_spaam'))
