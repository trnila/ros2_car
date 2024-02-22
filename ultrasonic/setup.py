from setuptools import setup

package_name = 'ultrasonic'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Trnka',
    maintainer_email='daniel.trnka@gmail.com',
    description='HC-SR04 Ultrasonic driver',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'node = ultrasonic.node:main',
        ],
    },
)
