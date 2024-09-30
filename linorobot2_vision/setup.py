from setuptools import setup

package_name = 'linorobot2_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thomas Tomow',
    maintainer_email='toto_san@live.com',
    description='Webcam node for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'linorobot2_vision = linorobot2_vision.vision_node:main'
        ],
    },
)