from subprocess import list2cmdline
from setuptools import setup

setup(
    name="plutolib",
    version="0.0.1",
    description="python based communication with pluto series mav",
    long_description="A library containing utilities used for communicating and controlling plutoX",
    author="Team 43",
    packages=[".plutolib"],
    install_requires=[
        "pyserial",
        "numpy",
    ],
    liscense="MIT",
    py_modules=["controller", "protocol", "kalman", "logger"],
)
