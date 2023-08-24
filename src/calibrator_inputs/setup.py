from setuptools import find_packages, setup

package_name = 'calibrator_inputs'

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
    maintainer='root',
    maintainer_email='legohead259@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rotary_encoder = calibrator_inputs.RotaryEncoderNode:main',
            'magnetometer = calibrator_inputs.MLX90393Node:main'
        ],
    },
)
