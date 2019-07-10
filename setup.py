from setuptools import setup, find_packages
import sys, subprocess

setup(name='primitive-planning',
      install_requires=[
          'numpy',  
          'trimesh',
          'python-fcl',
      ],
      description='Tactile Dexterity Project',
      author='Francois Hogan',
      url='https://github.com/fhogan/primitive_planning',
      author_email='',
version='0.1.0')
