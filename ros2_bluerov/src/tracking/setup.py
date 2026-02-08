from setuptools import find_packages, setup

package_name = 'tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/tracking_params.yaml']),
    ],
    install_requires=['setuptools', 'numpy>=1.17.3,<1.25.0', 'scipy'],
    zip_safe=True,
    maintainer='Maxime Lefevre',
    maintainer_email='maxime.lefevre@example.com',
    description='Détection des bords de la cage dans les données sonar',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracking_node = tracking.tracking_node:main',
            'blob_tracker_node = tracking.blob_tracker_node:main',
            'hough_lines_node = tracking.hough_lines_node:main',
            'csrt_tracker_node = tracking.csrt_tracker_node:main',
        ],
    },
)
