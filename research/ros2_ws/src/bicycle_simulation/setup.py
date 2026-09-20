from glob import glob
from setuptools import setup

name = "bicycle_simulation"
setup(
    name=name,
    version="0.2.0",
    packages=[name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + name]),
        ("share/" + name, ["package.xml"]),
        ("share/" + name + "/launch", glob("launch/*.launch.py")),
        ("share/" + name + "/config", glob("config/*")),
        ("share/" + name + "/urdf", glob("urdf/*")),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "plant = bicycle_simulation.plant:main",
            "estimator = bicycle_simulation.estimator:main",
            "controller = bicycle_simulation.controller:main",
            "recorder = bicycle_simulation.recorder:main",
        ]
    },
)
