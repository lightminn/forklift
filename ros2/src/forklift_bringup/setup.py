from glob import glob

from setuptools import setup

setup(
    name="forklift_bringup",
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/forklift_bringup"]),
        ("share/forklift_bringup", ["package.xml"]),
        ("share/forklift_bringup/config", glob("config/*.yaml")),
        ("share/forklift_bringup/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Forklift team",
    maintainer_email="noreply@example.com",
    description="Launch files and ROS parameters",
    license="MIT",
)
