from setuptools import setup

package_name = 'elfin_utils'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # si creas un launch más tarde:
        # ('share/' + package_name + '/launch', ['launch/joint_state_aligner.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='francisco',
    maintainer_email='flirizar@gmail.com',
    description='Utility nodes for Elfin robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_aligner = elfin_utils.joint_state_aligner:main',
            'trajectory_offset_shim = elfin_utils.trajectory_offset_shim:main',
        ],
    },
)
