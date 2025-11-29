from setuptools import setup
import os
from glob import glob

package_name = 'affichage'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy>=1.17.3,<1.25.0', 'PyQt5'],
    zip_safe=True,
    maintainer='Docking Team',
    maintainer_email='todo@todo.com',
    description='Interface de visualisation temps réel pour le système de docking',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_viewer = affichage.sonar_viewer:main',
        ],
    },
)
