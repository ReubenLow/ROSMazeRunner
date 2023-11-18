import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mark_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #to include launch folder in colcon build
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        #to include params folder in colcon build
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*params.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lusmse',
    maintainer_email='lusmse@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mark1 = mark_1.mark1:main',
        ],
    },
)
