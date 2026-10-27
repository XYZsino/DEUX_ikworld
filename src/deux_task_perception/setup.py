from glob import glob
from setuptools import find_packages, setup
setup(name='deux_task_perception', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/deux_task_perception']),
                  ('share/deux_task_perception', ['package.xml']),
                  ('share/deux_task_perception/config', glob('config/*.yaml')),
                  ('share/deux_task_perception/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools', 'numpy', 'scipy'], zip_safe=True,
      maintainer='XYZ', maintainer_email='dev@xyz.local', license='Apache-2.0',
      description='External ZED observation adapter',
      entry_points={'console_scripts': [
            'tracked_perception_node = deux_task_perception.tracked_perception_node:main',
            'tracked_detection_node = deux_task_perception.tracked_detection_node:main',
          'perception_node = deux_task_perception.merged_perception_node:main',
          'state_observation_node = deux_task_perception.state_observation_node:main',
          'barcode_node = deux_task_perception.barcode_node:main',
          'observation_node = deux_task_perception.observation_node:main',
          'detection_node = deux_task_perception.detection_node:main',
          'icp_node = deux_task_perception.icp_node:main']})
