from setuptools import find_packages, setup

package_name = 'robot_base'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='IIT Mandi Robotics Team',
    maintainer_email='team@iitmandi.ac.in',
    description='Differential-drive base controller, kinematics, and odometry node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_controller_node = robot_base.base_controller_node:main',
        ],
    },
)
