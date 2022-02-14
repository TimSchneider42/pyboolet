from setuptools import setup

with open("README.md", "r") as f:
    long_description = f.read()

setup(
    name="pyboolet",
    version="1.0.0",
    description="Object-oriented wrapper for pybullet.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/TimSchneider42/pyboolet",
    author="Tim Schneider",
    author_email="schneider@ias.informatik.tu-darmstadt.de",
    license="MIT",
    packages=["pyboolet"],
    install_requires=[
        "transformation3d==1.0.1",
        "pybullet"
    ],
    classifiers=[
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3.6",
    ],
)
