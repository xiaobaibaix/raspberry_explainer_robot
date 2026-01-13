from setuptools import find_packages, setup
from glob import glob

package_name = 'slam_cartographer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/config', glob("config/*.lua")),
        ('share/' + package_name+'/launch', glob("launch/*.launch.py")),
        ('share/' + package_name+'/maps', glob("maps/*.yaml")),
        ('share/' + package_name+'/maps', glob("maps/*.pgm")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huazhi',
    maintainer_email='2308043842@qq.com',
    description='TODO: Package description',
    license='MIT',
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
