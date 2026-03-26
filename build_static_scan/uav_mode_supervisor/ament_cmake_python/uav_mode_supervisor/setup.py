from setuptools import find_packages
from setuptools import setup

setup(
    name='uav_mode_supervisor',
    version='0.0.0',
    packages=find_packages(
        include=('uav_mode_supervisor', 'uav_mode_supervisor.*')),
)
