from setuptools import find_packages, setup

package_name = 'calibrator_plate'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/DRV8825.py', package_name+'/AMT22.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plate = calibrator_plate.CalibratorPlateNode:main',
            'encoder = calibrator_plate.AMT22Node:main',
            'magnetometer = calibrator_plate.MLX90393Node:main'
        ],
    },
)
