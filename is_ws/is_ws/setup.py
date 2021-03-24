import os
from glob import glob
from setuptools import setup

package_name = 'is_ws'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', [
    'resource/' + package_name
]))
data_files.append(('share/' + package_name, [
    'launch/navigation_cells_launch.py'
]))

data_files.append(('share/' + package_name + '/worlds', [
    'worlds/WorldMultipleCylinders.wbt', 'worlds/.WorldMultipleCylinders.wbproj', 
    'worlds/.world-multiple-cylinders.wbproj', 'worlds/.rat_experiment.wbproj'
]))
#data_files.append(('share/' + package_name + '/worlds', [
#    'worlds/lab3_task2.wbt', 'worlds/.lab3_task1_2_3.wbproj', 'worlds/.lab3_task1_2.wbproj',
#    'worlds/.lab3_task1.wbproj', 'worlds/.lab3_task2.wbproj', 'worlds/.lab3_task4.wbproj'
#]))

data_files.append(
    ('share/' + package_name + '/protos/icons', glob('protos/icons/*')))
data_files.append(
    ('share/' + package_name + '/worlds/textures', glob('worlds/textures/*')))
data_files.append(
    ('share/' + package_name + '/protos/textures', glob('protos/textures/*')))

data_files.append(('share/' + package_name, [
    'package.xml'
]))

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='zhinnen',
    maintainer_email='zhinnen@usf.edu',
    description='Independent Study Workplace',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'enable_robot = is_ws.slave:main',
            'navigation_cells = is_ws.master:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
