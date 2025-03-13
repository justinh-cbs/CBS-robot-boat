from setuptools import setup

package_name = "imu"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jetson",
    maintainer_email="jetson@todo.todo",
    description="IMU package with magnetometer calibration",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_node = imu.imu:main",
            "calibrate_magnetometer = imu.calibration_node:main",
            "imu_fusion = imu.imu_fusion_node:main",
        ],
    },
)
