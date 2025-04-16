from setuptools import find_packages, setup

package_name = 'auv_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'joystick_input_node = auv_controller.joystick_input_node:main',
            'sensor_node = auv_controller.sensor_node:main',
            'command_processor_node = auv_controller.command_processor_node:main',
        ],
    },
)
