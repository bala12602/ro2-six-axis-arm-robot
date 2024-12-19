from setuptools import setup
import os
from glob import glob

package_name = 'six_dof_arm'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Include package.xml file
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Include launch files
        ('share/' + package_name + '/launch',
            glob(os.path.join('launch', '*.py'))),  # All Python files in the launch directory

        # Include URDF files
        ('share/' + package_name + '/urdf',
            glob(os.path.join('urdf', '*.urdf'))),  # All URDF files in the urdf directory
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='6-DOF arm controller package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller = six_dof_arm.arm_controller:main',
        ],
    },
)
