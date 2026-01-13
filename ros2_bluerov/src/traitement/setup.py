from setuptools import find_packages, setup

package_name = 'traitement'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/traitement_params.yaml',
            'config/traitement_polar_params.yaml',
            'config/traitement_cartesian_params.yaml',
        ]),
    ],
    install_requires=['setuptools', 'numpy>=1.17.3,<1.25.0', 'scipy', 'opencv-python'],
    zip_safe=True,
    maintainer='Maxime Lefevre',
    maintainer_email='maxime.lefevre@example.com',
    description='Filtrage et prétraitement des données sonar',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traitement_node = traitement.traitement_node:main',
            'traitement_polar_node = traitement.traitement_polar_node:main',
            'traitement_cartesian_node = traitement.traitement_cartesian_node:main',
        ],
    },
)
