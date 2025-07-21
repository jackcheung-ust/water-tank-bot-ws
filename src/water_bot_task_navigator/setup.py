from setuptools import find_packages, setup

package_name = 'water_bot_task_navigator'

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
    maintainer='jack',
    maintainer_email='jackcheungyi@ust.hk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'water_bot_task_navigator = water_bot_task_navigator.water_bot_task_navigator:main',
        ],
    },
)
