from setuptools import find_packages, setup

package_name = 'drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/params', ['params/mapper_params_online_async.yaml', 'params/lifecycle_mgr_slam.yaml', 'params/nav2_params.yaml', 'params/rviz2.yaml']),  # Add this line
        ('share/' + package_name + '/launch', ['launch/sara_launch.py']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sara',
    maintainer_email='sara@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts': [
            'scan_listener = drive.scan_listener:main',
            'drive_node = drive.drive_node:main',
        ],  
    },
)
