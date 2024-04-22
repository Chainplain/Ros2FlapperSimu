from setuptools import setup
import os
import glob

package_name = 'clawed_flapper'
data_files = []
lib_name = 'lib'
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/clawed_flapper_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/clawed_flapper_in_ros2.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/clawed_flapper.urdf']))
data_files.append(('share/' + package_name + '/resource', glob.glob('resource' + '/*.npy')))
data_files.append(('share/' + package_name, ['package.xml']))

# data_folder = 'clawed_flapper'
# data_files.append(('share/' + package_name + '/launch', [os.path.join(data_folder, f) for f in os.listdir(data_folder)]))


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, lib_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='chainplain@163.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_clawed_flapper = clawed_flapper.my_clawed_flapper:main'
        ],
    },
)
