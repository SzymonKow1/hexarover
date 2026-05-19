from setuptools import find_packages, setup

package_name = 'cytron_driver'

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
    maintainer='bobik',
    maintainer_email='s198118@student.pg.edu.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cytron_node = cytron_driver.cytron_node:main',
            'imu_node = cytron_driver.imu_node:main',
        ],
    },
)
