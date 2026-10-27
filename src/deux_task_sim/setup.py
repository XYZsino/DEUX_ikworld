from setuptools import setup,find_packages
setup(name='deux_task_sim',version='0.1.0',packages=find_packages(),package_data={'deux_task_sim':['*.yaml']},
 data_files=[('share/ament_index/resource_index/packages',['resource/deux_task_sim']),('share/deux_task_sim',['package.xml','requirements.txt','requirements-vision.txt'])],
 install_requires=['setuptools'],zip_safe=True,maintainer='XYZ',maintainer_email='dev@xyz.local',license='Apache-2.0',
 description='MuJoCo ROS 2 driver and simulation-only sensor adapter',
 entry_points={'console_scripts':['serve_sim = deux_task_sim.serve:main','validate_transfer = deux_task_sim.validate:main','render_timeline = deux_task_sim.report:main']})
