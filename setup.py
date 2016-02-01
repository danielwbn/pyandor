from distutils.core import setup

setup(
    name='pyandor',
    version='0.1',
    author='Hamid Ohadi, Felix Benz, Skyenet, James Shubin, Martijn Schaafsma, Simon Dickreuter',
    #author_email='',
    packages=['Camera', 'Spectrograph'],
    description='Community-coded Python module for Andor spectrographs. This software is not associated with Andor. Use it at your own risk.',
    long_description=open('Readme.md').read(),
    requires=['python (>= 3.0)', 'numpy'],
)
