from setuptools import find_packages, setup

package_name = 'tum_converter'

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
    maintainer='artem-kondratew',
    maintainer_email='artemkondratev5@gmail.com',
    description='Package for convrting TUM datasets from ROS1 to ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'convert = tum_converter.tum_converter:main',
        ],
    },
)
    