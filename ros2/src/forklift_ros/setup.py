from setuptools import find_packages, setup

setup(
    name="forklift_ros",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/forklift_ros"]),
        ("share/forklift_ros", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Forklift team",
    maintainer_email="noreply@example.com",
    description="Synthetic Gazebo sensor observation validation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sensor_validator = forklift_ros.sensor_validator:main",
            "scene_capture = forklift_ros.scene_capture:main",
            "synthetic_tf = forklift_ros.synthetic_tf:main",
        ]
    },
)
