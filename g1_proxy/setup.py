from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'g1_proxy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='G1 Robot Controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'g1_node=g1_proxy.g1_node:main',
            'g1_online_ft_calibrator=g1_proxy.g1_online_ft_calibrator:main',
        ],
    },
)
