from setuptools import find_packages, setup

package_name = 'quadruped_gait'

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
    maintainer='jaewook',
    maintainer_email='jaewook@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gait_node = quadruped_gait.gait_node:main',
            'stm32_bridge = quadruped_gait.stm32_bridge:main'
        ],
    },
)
