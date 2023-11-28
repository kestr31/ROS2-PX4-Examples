from setuptools import find_packages, setup

package_name = 'px4'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kestrel',
    maintainer_email='kestrel@inha.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'takeoff = px4.takeoff.takeoff:main',
            'takeoff_att = px4.takeoff.takeoff_att:main'
        ],
    },
)
