from setuptools import find_packages, setup

package_name = 'robus_localization_pkg'

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
    maintainer='aswath',
    maintainer_email='asw3249s@hs-coburg.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robus_localization_node = robus_localization_pkg.robus_localization:main'
            ,'robus_test_localization_node = robus_localization_pkg.robus_test_localization:main'
            #'localization_node_lidar = robus_loc.localization_wih_lidar:main',
            #'record_data = robus_loc.record_data:main',
        ],
    },
)
