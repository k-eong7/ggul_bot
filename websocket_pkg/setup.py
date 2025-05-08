from setuptools import find_packages, setup

package_name = 'websocket_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='WebSocket to ROS2 topic publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ws_to_ros_publisher = websocket_pkg.ws_to_ros_publisher:main',
            'ik_ws_publisher = websocket_pkg.ik_ws_publisher:main',
        ],
    },
)
