from setuptools import setup, find_packages

setup(
    name="deux_task_core",
    version="0.2.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/deux_task_core"]),
        ("share/deux_task_core", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="XYZ",
    maintainer_email="dev@xyz.local",
    license="Apache-2.0",
    description="DEUX task implementation",
    entry_points={"console_scripts": []},
)
