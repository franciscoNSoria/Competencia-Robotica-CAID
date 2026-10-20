from setuptools import find_packages, setup
from glob import glob

package_name = 'caid_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='martin',
    maintainer_email='martinbravo23205@gmail.com',
    description='Camara y vision CAID',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'camara_node = caid_vision.camara_node:main',
            'vision_node = caid_vision.vision_node:main',
        ],
    },
)
