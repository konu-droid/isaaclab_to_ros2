import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'isaaclab_to_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files from the 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    # Add any required Python dependencies here
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A ROS 2 package for evaluating sim-to-real transfer.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This creates an executable named 'evaluator_node'
            'evaluator_node = isaaclab_to_ros2.sim_to_real_evaluator:main',
        ],
    },
)