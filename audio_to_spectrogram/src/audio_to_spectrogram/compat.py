import os
import subprocess
import warnings

import matplotlib


def check_matplotlib_version(
        python_version=os.getenv('ROS_PYTHON_VERSION')):
    if python_version == '2':
        python = 'python'
    else:
        python = 'python{}'.format(python_version)
    out = subprocess.check_output(
        ['dpkg', '-L', '{}-matplotlib'.format(python)])
    if isinstance(out, bytes):
        out = out.decode('utf-8')
    matplotlib_filelists = out.split()
    if os.path.dirname(matplotlib.__file__) in matplotlib_filelists:
        return True
    else:
        warnings.warn(
            "It looks like you are using pip's matplotlib. "
            "Apt's matplotlib is recommended.")
    return False
