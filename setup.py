from setuptools import setup

package_name = 'imitrob_templates'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        #('share/ament_index/resource_index/packages',
        #    ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='petr',
    maintainer_email='petr.vanc@cvut.cz',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'talker = my_great_rostwo_package.test:main',
            #'listener = my_great_rostwo_package.sub:main',
        ],
    },
)
