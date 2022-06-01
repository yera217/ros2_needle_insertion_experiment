from setuptools import setup
import glob

package_name = 'insertion_robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch/", glob.glob('*/*.launch.*')), # launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amiro',
    maintainer_email='yernar.zhetpissov@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_sim =  insertion_robot_sim.robot_sim:main',
        ],
    },
)
