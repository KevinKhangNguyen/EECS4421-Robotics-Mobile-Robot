import os
from glob import glob
from setuptools import setup


package_name = 'cpmr_ch4'

setup(
    name=package_name,
    version='3.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.json')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jenkin',
    maintainer_email='jenkin@yorku.ca',
    description='Code associated with Chapter 4 of CPMR 3rd Edition',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collect_lidar = cpmr_ch4.collect_lidar:main',
            'add_obstacle = cpmr_ch4.add_obstacle:main',
            'build_map = cpmr_ch4.build_map:main',
            'destroy_map = cpmr_ch4.destroy_map:main',
        ],
    },
)
