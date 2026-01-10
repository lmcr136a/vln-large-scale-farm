from setuptools import find_packages, setup

package_name = 'isaac_cartographer_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/isaac_cartographer.launch.py']),
        ('share/' + package_name + '/config', ['config/isaacsim.lua']),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koala',
    maintainer_email='koala@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
