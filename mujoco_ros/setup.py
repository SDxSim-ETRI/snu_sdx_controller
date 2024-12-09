from setuptools import setup
from glob import glob
import os

package_name = 'mujoco_ros'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/mujoco_ros']),  # 패키지 마커 파일
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
]

robots_path = 'robots'
for root, dirs, files in os.walk(robots_path):
    for file in files:
        relative_path = os.path.relpath(root, robots_path)
        install_path = os.path.join('share', package_name, 'robots', relative_path)
        data_files.append((install_path, [os.path.join(root, file)]))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoonjunheon',
    maintainer_email='yoonjh98@snu.ac.kr',
    description='Robot simulator packages with Mujoco & ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mobile_sim = mujoco_ros.mobile_sim:main',
            'manipulator_sim = mujoco_ros.manipulator_sim:main'
        ],
    },
)
