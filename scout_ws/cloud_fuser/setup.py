from setuptools import find_packages, setup

package_name = 'cloud_fuser'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    	('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    	('share/' + package_name, ['package.xml']),
    	('share/' + package_name + '/launch', ['launch/cloud_fuser_launch.py']),
    	('share/' + package_name + '/launch', ['launch/realsense_filter_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scoutmini2',
    maintainer_email='obrienaydan630@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'cloud_fuser_node = cloud_fuser.cloud_fuser_node:main',
        'realsense_filter_node = cloud_fuser.realsense_filter_node:main',
    	],
    },
)
