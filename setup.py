from setuptools import find_packages, setup

package_name = 'open_manipulator_tictactoe'

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
    maintainer='aallonas01',
    maintainer_email='alexandre.allonas@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = open_manipulator_tictactoe.camera_node:main',
            'controller_node = open_manipulator_tictactoe.controller_node:main',
            'ai_node = open_manipulator_tictactoe.ai_node:main'
        ],
    },
)
