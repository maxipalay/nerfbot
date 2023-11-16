from setuptools import find_packages, setup

package_name = 'pixel_to_world'

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
    maintainer='max',
    maintainer_email='maximilianopalay@gmail.com',
    description='Converts pixel coordinates to real world cooridnates form a RealSense camera.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pixel2world = pixel_to_world.node:main'
        ],
    },
)
