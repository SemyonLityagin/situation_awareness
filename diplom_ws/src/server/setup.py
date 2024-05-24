from setuptools import find_packages, setup
from glob import glob

package_name = 'server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + "/launch", glob('launch/*launch.[pxy][yma]*')),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/scenario", glob('scenario/server_data.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server_start = server.server:main',
        ],
    },
)
