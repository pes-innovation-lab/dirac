from setuptools import find_packages
from setuptools import setup

setup(
    name='dracon_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('dracon_msgs', 'dracon_msgs.*')),
)
