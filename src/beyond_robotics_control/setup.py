from setuptools import setup

package_name = 'beyond_robotics_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Beyond Robotics',
    maintainer_email='info@beyondrobotics.com',
    description='Beyond Robotics differential drive control package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'beyond_diffdrive = beyond_robotics_control.beyond_diffdrive:main',
            'lane_detection = beyond_robotics_control.lane_detection:main',
            'keyboard_control = beyond_robotics_control.keyboard_control:main',
            'qr_detection = beyond_robotics_control.qr_detection:main',
            'robot_control = beyond_robotics_control.robot_control:main',
            'keyboard_input = beyond_robotics_control.keyboard_input:main',
            'aruco_detection = beyond_robotics_control.aruco_detection:main',
            'dual_camera = beyond_robotics_control.dual_camera:main',
            'aruco_path_following = beyond_robotics_control.aruco_path_following:main',
            'demo_robot_control = beyond_robotics_control.demo_robot_control:main',
        ],
    },
)
