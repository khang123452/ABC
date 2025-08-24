from setuptools import setup

package_name = 'robopepp_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robopepp_bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/robopepp_ur3.yaml', 'config/intrinsics.yaml', 'config/extrinsics.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='User',
    maintainer_email='user@example.com',
    description='RoboPEPP inference and RTDE bridge for two UR3 arms.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'robopepp_node = robopepp_bridge.robopepp_node:main',
            'rtde_left_node = robopepp_bridge.rtde_left_node:main',
            'rtde_right_node = robopepp_bridge.rtde_right_node:main',
            'extrinsic_calib_node = robopepp_bridge.extrinsic_calib_node:main'
        ],
    },
)
