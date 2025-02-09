from setuptools import find_packages, setup

package_name = 'tc'

setup(
    name=package_name,
    version='0.0.0',
    packages=["tc", "tc/include"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wy',
    maintainer_email='realmeteny10@gmail.com',
    description='Target Chasing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'center_publisher = tc.center_publisher:main',
            'center_subscriber = tc.center_subscriber:main',
        ],
    },
)
