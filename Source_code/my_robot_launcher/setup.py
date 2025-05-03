from setuptools import setup

package_name = 'my_robot_launcher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/robot_system_launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='System launcher for robot nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)