from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['dilly_auto_driving'],
    package_dir={'': 'src'}
)

setup(**d)