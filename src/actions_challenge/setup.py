from setuptools import find_packages, setup

package_name = 'actions_challenge'

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
    maintainer='lijovijayan',
    maintainer_email='lijovijayan00@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot_server = actions_challenge.robot_server_node:main",
            "robot_client = actions_challenge.robot_client_node:main"
        ],
    },
)
