from glob import glob

from setuptools import setup

package_name = "flexiv_handeye_calib"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="v2ck3",
    maintainer_email="v2ck3@example.com",
    description="Eye-in-hand calibration for Flexiv Rizon arms",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "calibrate = flexiv_handeye_calib.calibrate_node:main",
            "validate = flexiv_handeye_calib.validate_node:main",
            "handeye_solve = flexiv_handeye_calib.solve_cli:main",
            "doctor = flexiv_handeye_calib.doctor:main",
            "pose_delta = flexiv_handeye_calib.pose_delta:main",
            "auto_move = flexiv_handeye_calib.auto_move:main",
        ],
    },
)
