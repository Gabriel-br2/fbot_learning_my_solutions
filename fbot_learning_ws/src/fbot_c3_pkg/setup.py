from setuptools import find_packages, setup

package_name = 'fbot_c3_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel',
    maintainer_email='souza.gabriel.0210@gmail.com',
    description='ROS 2 package for the turtle challenge 3',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtle_control_c3_node = fbot_c3_pkg.sm_turtle_control:main',
        ],
    },
)
