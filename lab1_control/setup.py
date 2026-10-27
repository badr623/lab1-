from setuptools import find_packages, setup

package_name = 'lab1_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Control nodes for Lab 1 (goToPose, reset, square)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'goto_pose_server = lab1_control.goto_pose_server:main',
            'reset_service = lab1_control.reset_service:main',
            'square_server = lab1_control.square_server:main',
        ],
    },
)
