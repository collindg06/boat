from setuptools import find_packages, setup

package_name = 'my_pubsub'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'talker = my_pubsub.mytalker:main',
                'listener = my_pubsub.mylistener:main',
                'testmotor = my_pubsub.testmotor:main',
                'listenmotor = my_pubsub.listenmotor:main',
                'talkermotor = my_pubsub.talkermotor:main',
                'undt  = my_pubsub.undt:main',
                'undl = my_pubsub.undl:main',
                'read_PWM = my_pubsub.read_PWM:main',
                'boatt = my_pubsub.boatt:main',
                'boatl = my_pubsub.boatl:main',
        ],
    },
)
