from setuptools import find_packages, setup

package_name = 'mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/mission_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maxime Lefevre',
    maintainer_email='maxime.lefevre@example.com',
    description='Machine d\'états pour la mission de docking',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = mission.mission_node:main',
        ],
    },
)
