from setuptools import setup

package_name = "aerovio_adapter"

setup(
    name=package_name,
    version="2.0.0",
    py_modules=["adapter"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="iamakhilan",
    maintainer_email="akhilan@users.noreply.github.com",
    description="ROS 2 live telemetry adapter for the AeroVIO Ground Station.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "adapter = adapter:main",
        ],
    },
)
