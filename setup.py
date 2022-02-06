import os
from setuptools import setup


# Utility function to read the README file.
# Used for the long_description.  It's nice, because now 1) we have a top level
# README file and 2) it's easier to type in the README file than to put a raw
# string in below ...
def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()


setup(
    name="autonomousPlayer",
    version="0.0.1",
    author="Nathan Sala",
    author_email="natouda@gmail.com",
    description="Autonomous player for a robotic game",
    license="BSD",
    keywords="robotic autonomous player",
    url="https://github.com/natsala13/comotion_autonomous_player",
    packages=['autonomous_player'],
    long_description=read('README.md'),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Topic :: Algo",
        "License :: OSI Approved :: BSD License",
    ],
)
