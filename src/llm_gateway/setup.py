import os
from glob import glob
from setuptools import setup

package_name = 'llm_gateway'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
    ],
    install_requires=['setuptools', 'jsonschema'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='LLM Gateway layer for Yaskawa GP4 robot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_gateway_node = llm_gateway.gateway_node:main',
        ],
    },
)
