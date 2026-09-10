from setuptools import setup
from glob import glob

package_name = 'a4_neil'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Neil',
    maintainer_email='neilgeorge03@gmail.com',
    description='MFE A4 Neil-side sensor sim + grader.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_sim_node = a4_neil.sensor_sim_node:main',
            'grader = a4_neil.grader:main',
        ],
    },
)
