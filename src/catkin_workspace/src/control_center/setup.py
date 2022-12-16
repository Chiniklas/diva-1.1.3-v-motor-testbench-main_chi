from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['control_center'],
    package_dir={'': 'src'},
    scripts=['scripts/control_center']
)

setup(**d)