from glob import glob

from setuptools import find_packages, setup

package_name = 'casabot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adrian Melendez',
    maintainer_email='70454832+AdrianMelendez@users.noreply.github.com',
    description='Map a house with a 2D lidar, then drive to places you name.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_driver = casabot.base_driver:main',
            'places = casabot.places:main',
        ],
    },
)
