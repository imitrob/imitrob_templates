from setuptools import setup

package_name = 'imitrob_templates'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
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
            'template_execution_node = imitrob_templates.template_execution_node:main',
            'send_template = imitrob_templates.manual_tests.send_sample_hricommand:main',
            'send_templates_1 = imitrob_templates.manual_tests.send_sample_hricommand:send_templates_1',
            'send_moves = imitrob_templates.manual_tests.send_sample_move:main',
            'send_move = imitrob_templates.manual_tests.send_sample_move:send_move',
            'homing = imitrob_templates.manual_tests.send_sample_move:homing',
            'close = imitrob_templates.manual_tests.send_sample_move:close',
            'open = imitrob_templates.manual_tests.send_sample_move:open'
        ],
    },
)
