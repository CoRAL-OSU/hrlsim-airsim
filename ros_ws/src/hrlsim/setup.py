from setuptools import setup

package_name = 'hrlsim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'hrlsim/airsim', 'hrlsim/controller', 'hrlsim/drone', 'hrlsim/opt', 'hrlsim/traj', 'hrlsim/utility'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['pkgutil','setuptools', 'control', 'cmake', 'slycot', 'numpy', 'msgpack-rpc-python', 'pymap3d'],
    zip_safe=True,
    maintainer='cthornton',
    maintainer_email='collin.thornton@okstate.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
