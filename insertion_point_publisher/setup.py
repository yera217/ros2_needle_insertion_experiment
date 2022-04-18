from setuptools import setup
import glob

package_name = 'insertion_point_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/config/", glob.glob('config/*.yaml')), # configuration files
        ('share/' + package_name + "/launch/", glob.glob('*/*.launch.*')), # launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dimitri Lezcano',
    maintainer_email='dlezcan1@jhu.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'insertion_point_publisher = {package_name}.insertion_point_publisher:main'
        ],
    },
)
