from setuptools import find_packages, setup

package_name = 'pf_grapher'

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
    maintainer='s2911476',
    maintainer_email='s2911476@ed.ac.uk',
    description='Grapher',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pf_grapher = pf_grapher.pf_grapher:main'
        ],
    },
)
