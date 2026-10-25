from setuptools import setup, find_packages

setup(
    name="deux_task_runtime",
    version="0.2.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/deux_task_runtime"]),
        ("share/deux_task_runtime", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="XYZ",
    maintainer_email="dev@xyz.local",
    license="Apache-2.0",
    description="DEUX task implementation",
    entry_points={
        "console_scripts": [
            "task_node = deux_task_runtime.task_node:main",
            "motion_planner_node = deux_task_runtime.motion_planner_node:main",
            "execution_node = deux_task_runtime.execution_node:main",
        ]
    },
)
