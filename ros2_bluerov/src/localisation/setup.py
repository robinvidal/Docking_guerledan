from setuptools import find_packages, setup

package_name = 'localisation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/localisation_params.yaml']),
    ],
    install_requires=['setuptools', 'numpy>=1.17.3,<1.25.0'],
    zip_safe=True,
    maintainer='Maxime Lefevre',
    maintainer_email='maxime.lefevre@example.com',
    description='Calcul de la pose relative du ROV par rapport à la cage',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localisation_node = localisation.localisation_node:main',
        ],
    },
)
