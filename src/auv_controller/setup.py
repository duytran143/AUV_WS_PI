from setuptools import find_packages, setup
from glob import glob

package_name = 'auv_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auv',
    maintainer_email='auv@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = auv_controller.sensor_node:main',
            'camera_stream_node     = auv_controller.camera_stream_node:main',
            'gui_command_node     = auv_controller.gui_command_node:main',
            'user_input = auv_controller.user_input:main',
            'user_input_temp = auv_controller.user_input_temp:main',
            'ROV_main = auv_controller.ROV_main:main',
            'ROV_main_temp = auv_controller.ROV_main_temp:main',
            'motor_driver_node = auv_controller.motor_driver_node:main'
        ],
    },
)
