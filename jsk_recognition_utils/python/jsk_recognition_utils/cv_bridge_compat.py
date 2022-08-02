import importlib
import os
import sys


# cv_bridge_python3 import
if os.environ['ROS_PYTHON_VERSION'] == '3':
    cv_bridge = importlib.import_module("cv_bridge")
    CvBridge = getattr(cv_bridge, 'CvBridge')
else:
    ws_python3_paths = [p for p in sys.path if 'devel/lib/python3' in p]
    if len(ws_python3_paths) == 0:
        # search cv_bridge in workspace and append
        ws_python2_paths = [
            p for p in sys.path if 'devel/lib/python2.7' in p]
        for ws_python2_path in ws_python2_paths:
            ws_python3_path = ws_python2_path.replace('python2.7', 'python3')
            if os.path.exists(os.path.join(ws_python3_path, 'cv_bridge')):
                ws_python3_paths.append(ws_python3_path)
        if len(ws_python3_paths) == 0:
            opt_python3_path = '/opt/ros/{}/lib/python3/dist-packages'.format(
                os.getenv('ROS_DISTRO'))
            sys.path = [opt_python3_path] + sys.path
            cv_bridge = importlib.import_module("cv_bridge")
            CvBridge = getattr(cv_bridge, 'CvBridge')
            sys.path.remove(opt_python3_path)
        else:
            sys.path = [ws_python3_paths[0]] + sys.path
            cv_bridge = importlib.import_module("cv_bridge")
            CvBridge = importlib.import_module("cv_bridge.CvBridge")
            CvBridge = getattr(cv_bridge, 'CvBridge')
            sys.path.remove(ws_python3_paths[0])
    else:
        cv_bridge = importlib.import_module("cv_bridge")
        CvBridge = getattr(cv_bridge, 'CvBridge')
