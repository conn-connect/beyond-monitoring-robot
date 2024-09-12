from setuptools import setup

package_name = 'beyond_robotics_investors_demo'

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
            'beyond_diffdrive = beyond_robotics_investors_demo.beyond_diffdrive:main',
            'lane_detection = beyond_robotics_investors_demo.lane_detection:main',
            'keyboard_control = beyond_robotics_investors_demo.keyboard_control:main',
            'qr_detection = beyond_robotics_investors_demo.qr_detection:main',
            'robot_control = beyond_robotics_investors_demo.robot_control:main',
            'keyboard_input = beyond_robotics_investors_demo.keyboard_input:main',
            'aruco_detection = beyond_robotics_investors_demo.aruco_detection:main',
            'dual_camera = beyond_robotics_investors_demo.dual_camera:main'
        ],
    },
)
