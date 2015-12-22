#!/usr/bin/env python

from setuptools import setup, Extension, find_packages

source_files = [
    "src/Fusion.cpp",
    "src/pyglue.cpp",
    "src/MadgwickAHRS.c",
    "src/MadgwickQuaternion.c",
]

glue_module = Extension(
    "glue",
    source_files,
    libraries=["boost_python"],
    include_dirs=["include"])

setup(
    name="sensorfusion",
    version="0.0.4",
    description="python sensor fusion, copied from Android",
    author="Amir Sadoughi",
    author_email="sensorfusion@princehonest.com",
    url="https://github.com/asadoughi/sensorfusion",
    packages=find_packages(),
    ext_package="sensorfusion",
    ext_modules=[glue_module]
)
