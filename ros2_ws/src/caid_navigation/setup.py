from glob import glob

from setuptools import find_packages, setup

package_name = 'caid_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='martin',
    maintainer_email='martinbravo23205@gmail.com',
    description='Navegación: mapa de celdas y exploración del laberinto',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'explorador = caid_navigation.explorador:main',
            'cubo_simulado = caid_navigation.cubo_simulado:main',
            'carga_simulada = caid_navigation.carga_simulada:main',
        ],
    },
)
