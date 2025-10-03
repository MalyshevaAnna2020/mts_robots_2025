from setuptools import find_packages, setup
from glob import glob

package_name = 'controller'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/launch', glob('launch/*')))
data_files.append(('share/' + package_name + '/resource', glob('resource/*')))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anna',
    maintainer_email='almalysheva10@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=[],
    entry_points={
        'console_scripts': [
            "telemetry_bridge = controller.telemetry_bridge:main",
        ],
    },
)
