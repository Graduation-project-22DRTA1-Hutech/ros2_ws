from setuptools import setup

package_name = 'xarm_yolo_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='xArm YOLO control via joystick',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joy_xarm = xarm_yolo_joy.joy_xarm_node:main',
        ],
    },
)

