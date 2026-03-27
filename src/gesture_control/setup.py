from setuptools import find_packages, setup
import os

package_name = 'gesture_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[ (os.path.join('lib', package_name), ['scripts/run_gesture_pub_venv.sh']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jamie1',
    maintainer_email='jamie1@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'gesture_pub = gesture_control.gesture_pub:main',
		'gesture_to_jointstates = gesture_control.gesture_to_jointstates:main',
		'moveit_cartesian_demo = gesture_control.moveit_cartesian_demo:main',
        ],
    },
)
