from setuptools import setup

package_name = 'precision_landing_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/precision_landing.launch.py']),
        ('share/' + package_name + '/config', ['config/precision_landing_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Precision landing package for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'precision_lander_node = precision_landing_ros2.precision_lander_node:main',
            'minimal_timer_test = precision_landing_ros2.minimal_timer_test:main',
        ],
    },
)
