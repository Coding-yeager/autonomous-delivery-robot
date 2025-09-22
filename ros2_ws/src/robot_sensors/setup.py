from setuptools import find_packages, setup

package_name = 'robot_sensors'

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
    description='Sensor drivers for MPU6050 IMU, Holybro NEO-M9N GNSS, and Health Monitor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpu6050_node = robot_sensors.mpu6050_node:main',
            'gps_node = robot_sensors.gps_node:main',
            'sensor_health_monitor = robot_sensors.sensor_health_monitor:main',
        ],
    },
)
