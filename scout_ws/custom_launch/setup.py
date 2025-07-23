from setuptools import find_packages, setup

package_name = 'custom_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name + '/launch', [
    'launch/Custom_Slam_Launch.py',
    'launch/Custom_Nav2_Launch.py',
    'launch/Custom_Nav2_Map_Launch.py',
    'launch/Custom_Slam_Offline_Launch.py',
    'launch/Custom_Slam_RS_Launch.py',
    'launch/Custom_Nav2_RS_Launch.py',
    'launch/Custom_Slam_ALL_Launch.py',
    'launch/Custom_Nav2_ALL_Launch.py',
    'launch/Custom_Nav2_ALL_Map_Launch.py'
	]),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scoutmini2',
    maintainer_email='obrienaydan630@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

