from glob import glob

from setuptools import setup

package_name = "rl_deploy"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/models", glob("models/*.onnx")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="iclab",
    maintainer_email="5454sky@gmail.com",
    description="RL policy 真機部署與校正",
    license="MIT",
    entry_points={
        "console_scripts": [
            "calib = rl_deploy.calib_node:main",
            "dxl = rl_deploy.dxl_tool:main",
            "driver_patched = rl_deploy.driver_patched:main",
            "policy_node = rl_deploy.policy_node:main",
        ],
    },
)
