# -*- coding: utf-8 -*-
from setuptools import setup
import os
from glob import glob
from generate_parameter_library_py.setup_helper import generate_parameter_module

package_name = "sine_wave_py"

generate_parameter_module(
    "sine_wave_parameters", "config/sine_wave_parameters.yaml"  # Python  # YAML
)


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="ziou.hu@tum.de",
    description="Python version of sine_wave_cpp package",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sine_wave_publisher_node = sine_wave_py.sine_wave_publisher_node:main",
            "sine_wave_receiver_node = sine_wave_py.sine_wave_receiver_node:main",
        ],
    },
)
