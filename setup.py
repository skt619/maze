from setuptools import find_packages, setup

package_name = 'ros_package'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/ros_launch.py']))

data_files.append(('share/' + package_name + '/worlds', [
    'worlds/maze.wbt', 
]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/resource', [
    'resource/turtlebot_webots.urdf',
    'resource/ros2control.yml',
]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='Shafkat Hassan',
    maintainer_email='shassan6@crimson.ua.edu',
    description='CS560 Autonomous Indoor Search',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'],    
        'console_scripts': ['my_controller = ros_package.my_controller:main']
    },

)

