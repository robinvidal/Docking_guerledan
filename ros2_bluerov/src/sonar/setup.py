from setuptools import find_packages, setup

package_name = 'sonar'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/sonar_params.yaml']),
    ],
    install_requires=['setuptools', 'numpy>=1.17.3,<1.25.0', 'scipy'],
    zip_safe=True,
    maintainer='Maxime Lefevre',
    maintainer_email='maxime.lefevre@example.com',
    description='Driver et mock pour sonar Oculus M750d',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_node = sonar.sonar_node:main',
            'sonar_mock = sonar.sonar_mock:main',
        ],
    },
)
