from setuptools import find_packages, setup

package_name = 'navigation_examples'

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
    description='ROS2-python examples to navigate with Avular its Origin',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'get_velocity = navigation_examples.get_velocity:main',
        	'set_velocity = navigation_examples.set_velocity:main',
            'get_2dpose = navigation_examples.get_2dpose:main',
        ],
    },
)
