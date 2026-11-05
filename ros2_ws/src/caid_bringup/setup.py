from glob import glob

from setuptools import find_packages, setup

package_name = 'caid_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.*')),
        ('share/' + package_name + '/config/sim', glob('config/sim/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='martin',
    maintainer_email='martinbravo23205@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scan_to_range = caid_bringup.scan_to_range:main',
            'odom_ruedas = caid_bringup.odom_ruedas:main',
            'odom_ticks = caid_bringup.odom_ticks:main',
            'sonar_bridge = caid_bringup.sonar_bridge:main',
            'calibracion_robot = caid_bringup.calibracion_robot:main',
        ],
    },
)
