from setuptools import setup
from glob import glob

package_name = 'a4_student'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='MFE A4 student template.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dead_reckoning_node = a4_student.dead_reckoning_node:main',
            'ekf_node = a4_student.ekf_node:main',
        ],
    },
)
