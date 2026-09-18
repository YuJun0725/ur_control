from glob import glob
import os

from setuptools import find_packages, setup


package_name = "ur7e_motion"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wenqin",
    maintainer_email="wenqin@todo.todo",
    description="Reusable MoveItPy motion control for a UR7e using ROS 2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fixed_task = ur7e_motion.fixed_task:main",
            "motion_command = ur7e_motion.motion_command:main",
            "obstacle_demo = ur7e_motion.obstacle_demo:main",
            "pick_place_demo = ur7e_motion.pick_place_demo:main",
            "workspace_pick_place_demo = "
            "ur7e_motion.workspace_pick_place_demo:main",
        ],
    },
)
