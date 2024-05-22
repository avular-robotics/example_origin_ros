from setuptools import find_packages, setup

package_name = 'perception_examples'

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
    maintainer='bep',
    maintainer_email='j.sijs@avular.com',
    description='Example on the perception of the robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_closest = perception_examples.get_velocity:main',
        ],
    },
)
