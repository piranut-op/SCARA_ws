from setuptools import find_packages, setup

package_name = 'SCARA_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/launch.py',
          'launch/scara_hw_bridge.launch.py',
          'launch/perception_to_ik.launch.py',
          'launch/cam_pick_and_place.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='piranut-op',
    maintainer_email='piranutphlong@gmail.com',
    description='Hardware bridge (ODrive and MKS over SocketCAN) plus Python IK/FK, joint-state, and RealSense+YOLO bottle-cap detection nodes for the SCARA robot.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
        	'motor_selection = SCARA_pkg.motor_selection:main',
            'position = SCARA_pkg.position:main',
            'positionradient = SCARA_pkg.positionradient:main',
            'fkpos = SCARA_pkg.fkpos:main',
            'newposition = SCARA_pkg.newposition:main',
            'ikpos = SCARA_pkg.ikpos:main',
            'camdetect = SCARA_pkg.camdetect:main',
            'mks_ee = SCARA_pkg.mks_ee:main',
            'joint_states_bridge = SCARA_pkg.joint_states_bridge:main',
            'setup_scara_gui = SCARA_pkg.setup_scara_gui:main',
            'detect_bottle_cap = SCARA_pkg.detect_bottle_cap:main',
            'cap_to_target = SCARA_pkg.cap_to_target:main',
            'pick_and_place_cam = SCARA_pkg.pick_and_place_cam:main',
        ],
    },
)
