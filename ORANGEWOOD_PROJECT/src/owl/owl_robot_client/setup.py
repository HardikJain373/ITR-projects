#!/usr/bin/env python3
from setuptools import setup, find_packages

setup(
    name="owl_robot_client",
    version="1.0.0",
    description="A python client library to operate OWL Robot over LAN.",
    author="RajKumar Gupta",
    author_email="rajkumar.g@orangewood.co",
    license="Orangewood Labs Inc. - For internal Use Only",
    packages=find_packages(exclude=["tests", "docs", "examples"]),
    install_requires=["msgpack", "grpcio", "grpcio-tools", "protobuf>=4.21.6"],
)
