from setuptools import setup

package_name = 'odom_to_tf_py'

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
    description='Convert nav_msgs/Odometry to TF (odom -> base_link)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'wheel_odom_node = odom_to_tf_py.wheel_odom_node:main',
    ],

    },
)

