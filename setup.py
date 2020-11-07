import os
import pathlib
import platform
from distutils.core import setup

from setuptools import find_packages

this_directory = pathlib.Path(__file__).parent.resolve()
os_name = platform.system()

with open(os.path.join(this_directory, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

with open(os.path.join(this_directory, 'requirements.txt'), encoding='utf-8') as f:
    requirements = f.read()


setup(
    name='pedro',
    version='0.01',
    packages=find_packages(),
    author='Caleb Hyde',
    author_email='caleb.hyde@gmail.com',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/clebio/spot-micro-quadruped',
    install_requires=requirements,
    entry_points='''
        [console_scripts]
        pedro=src.cli:cli
    ''',
    include_package_data=True,
    keywords=['robot', 'quadruped', 'pybullet', 'spotmicro', 'pedro'],
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Framework :: Robot Framework :: Library',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3.8']
)
