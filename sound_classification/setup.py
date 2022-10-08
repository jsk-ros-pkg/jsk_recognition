from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup


d = generate_distutils_setup(
    packages=['sound_classification'],
    package_dir={'': 'src'}
)

setup(**d)
