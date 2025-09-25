from setuptools import find_packages
from setuptools import setup

setup(
    name='ch4_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('ch4_interfaces', 'ch4_interfaces.*')),
)
