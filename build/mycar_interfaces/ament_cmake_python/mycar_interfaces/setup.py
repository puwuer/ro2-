from setuptools import find_packages
from setuptools import setup

setup(
    name='mycar_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('mycar_interfaces', 'mycar_interfaces.*')),
)
