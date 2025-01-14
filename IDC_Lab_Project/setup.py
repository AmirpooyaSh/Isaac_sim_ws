from setuptools import setup, find_packages

setup(
    name='ap_import_isaac',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[],  # List dependencies here if any
    include_package_data=True,
    description='A Built-in Library to integrated Isaac Sim and CuRobo for any Robotic Arm Instances',
    author='Amirpooya Shirazi',
    author_email='a.shirazi@ufl.edu',
)