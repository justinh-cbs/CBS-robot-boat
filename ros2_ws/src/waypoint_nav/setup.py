from setuptools import setup

package_name = "waypoint_nav"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "waypoint_nav_interfaces"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="your_email@example.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "waypoint_navigator = waypoint_nav.waypoint_navigator:main"
        ],
    },
)
