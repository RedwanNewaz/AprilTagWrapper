import setuptools
import pathlib

import pkg_resources
import setuptools

with pathlib.Path('requirements.txt').open() as requirements_txt:
    install_requires = [
        str(requirement)
        for requirement
        in pkg_resources.parse_requirements(requirements_txt)
    ]

setuptools.setup(
    name="AprilTagWrapper",
    version="0.0.1",
    author="RedwanNewaz",
    author_email="redwan06me@gmail.com",
    description="April Tag detection and tracking with Extended Kalman Filter",
    long_description="The basic April Tag functionality has been improved with Opencv interface and Extended Kalman Filter",
    long_description_content_type="text/markdown",
    url="https://github.com/RedwanNewaz/AprilTagWrapper",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    packages=setuptools.find_packages(),
    python_requires='>=3.6',
    install_requires=install_requires,
)