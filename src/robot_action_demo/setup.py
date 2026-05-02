from setuptools import setup

package_name = "robot_action_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/task_launch_map.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="miao",
    maintainer_email="miao@miao.com",
    description="Minimal ROS 2 action server/client demo for robot dispatch.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dispatch_server = robot_action_demo.dispatch_server:main",
            "dispatch_client = robot_action_demo.dispatch_client:main",
        ],
    },
)
