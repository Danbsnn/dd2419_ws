from setuptools import find_packages, setup

package_name = 'random_goal_generator'

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
    maintainer='sleepy7',
    maintainer_email='daniel.purroy02@gmail.com',
    description='generates random pose goal in rectangle 1.5m x 2m',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'random_goal_generator = random_goal_generator.random_goal_generator:main',
        ],
    },
)
