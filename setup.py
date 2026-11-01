from setuptools import find_packages, setup

package_name = 'crane_plus_automator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='',
    description='Fetch cube pose from Gazebo and grasp it with the crane_plus gripper',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'demo1 = crane_plus_automator.demo1:main',
            'reset_state = crane_plus_automator.reset_state:main',
        ],
    },
)
