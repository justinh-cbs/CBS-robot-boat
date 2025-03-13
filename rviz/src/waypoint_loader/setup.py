from setuptools import setup

package_name = "waypoint_loader"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="Loads waypoints from CSV files",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "waypoint_loader = waypoint_loader.waypoint_loader:main",
            "geojson_converter = waypoint_loader.geojson_converter:main",
        ],
    },
)
