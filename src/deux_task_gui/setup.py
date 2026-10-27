from setuptools import setup, find_packages
setup(name='deux_task_gui', version='0.1.0', packages=find_packages(),
    package_data={'deux_task_gui': ['index.html']},
    data_files=[('share/ament_index/resource_index/packages', ['resource/deux_task_gui']), ('share/deux_task_gui', ['package.xml']), ('share/deux_task_gui/launch', ['launch/control.launch.py'])],
    install_requires=['setuptools'], zip_safe=True, maintainer='XYZ', maintainer_email='dev@xyz.local',
    description='Remote browser controls for ROS 2 task actions', license='Apache-2.0',
    entry_points={'console_scripts': ['task_gui = deux_task_gui.node:main']})
