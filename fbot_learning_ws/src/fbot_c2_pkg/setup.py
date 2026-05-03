from setuptools import find_packages, setup

package_name = 'fbot_c2_pkg'

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
    description='ROS 2 package for the turtle challenge 2',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtle_control_c2_node = fbot_c2_pkg.turtle_control_node:main'
        ],
    },
)
