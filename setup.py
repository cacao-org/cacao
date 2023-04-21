'''
camstack setup.py

deps:
    libtmux
    pyMilk
    docopt
'''

from setuptools import setup
from setuptools.command.install import install
from setuptools.command.develop import develop
from subprocess import check_call

import os


class PreInstallCommand(develop):

    def run(self):
        # Add custom make commands here !
        # Migration of compiled grabbers to hardwaresecrets repo.
        #os.system('cd src; ./compile_edt; cd ..')
        #os.system('cd src; ./compile_dcamusb; cd ..')
        develop.run(self)

scripts = [
]

setup(
        name='pycacao',
        version='1.0',
        description='Python AO tools over CACAO',
        long_description='No long description',
        author='Vincent Deo & Olivier Guyon',
        author_email='vdeo@naoj.org',
        url="http://www.github.com/cacao-org/cacao",
        packages=['pycacao'],  # same as name
        install_requires=['numpy', 'astropy'],
        scripts=scripts,
        cmdclass={'develop': PreInstallCommand},
)
