from setuptools import find_packages, setup

package_name = 'macvo_vis'

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
    maintainer='yutian',
    maintainer_email='markchenyutian@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_vis = macvo_vis.same_device:main',
            'cross_vis = macvo_vis.cross_device:main'
        ],
    },
)
