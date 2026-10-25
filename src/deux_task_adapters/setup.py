from setuptools import setup, find_packages

setup(
    name="deux_task_adapters",
    version="0.2.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/deux_task_adapters"]),
        ("share/deux_task_adapters", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="XYZ",
    maintainer_email="dev@xyz.local",
    license="Apache-2.0",
    description="DEUX task implementation",
    entry_points={
        "console_scripts": [
            "arm_bridge_node = deux_task_adapters.arm_bridge_node:main",
            "hand_bridge_node = deux_task_adapters.hand_bridge_node:main",
        ]
    },
)
