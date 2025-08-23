from setuptools import setup

package_name = 'bldc_motor_gui'

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
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='BLDC Motor GUI with PyQt5 and pyqtgraph',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bldc_motor_gui = bldc_motor_gui.gui:main',
        ],
    },
)
