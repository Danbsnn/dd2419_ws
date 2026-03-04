from setuptools import find_packages, setup

package_name = 'decision_maker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elise',
    maintainer_email='elise.goffaux@gmail.com',
    description='Node for frontier exploration',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'decision_maker_node = decision_maker.decision_maker.decision_maker_node:main',
        ],
    },
)