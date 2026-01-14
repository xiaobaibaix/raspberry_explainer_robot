from setuptools import find_packages, setup
from glob import glob
package_name = 'nav2_app'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/config', glob("config/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
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
            "nav2_through_pose = nav2_app.nav2_through_pose:main",
            "nav2_simple_one = nav2_app.nav2_simple_one:main",
            "nav2_set = nav2_app.nav2_set:main",
        ],
    },
)
