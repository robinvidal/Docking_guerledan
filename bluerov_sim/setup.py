from setuptools import setup

package_name = 'bluerov_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='BlueROV simple simulator + controller',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'simulator_node = bluerov_sim.simulator_node:main',
        'controller_node = bluerov_sim.controller_node:main',
    ],},
)
