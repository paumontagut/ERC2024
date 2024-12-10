from setuptools import find_packages, setup

package_name = 'rover_supervisor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diego',
    maintainer_email='al426641@uji.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'supervisor_node = rover_supervisor.supervisor_node:main',
            'supervisor_node_persistent_terminal = rover_supervisor.supervisor_node_persistent_terminal:main',
        ],
    },
)
