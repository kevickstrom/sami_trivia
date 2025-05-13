import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'sami_trivia'
os.environ["PYTHON_EXECUTABLE"] = "/home/user/Desktop/trivia_ws/.venv/bin/python"
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.yaml'))),
        (os.path.join('share', package_name, 'assets'), glob('assets/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevickstrom',
    maintainer_email='vickskyl@oregonstate.edu',
    description='ROB421 Trivia Interaction Pkg',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trivia = sami_trivia.triviaGame:createGame',
            'sami_control = sami_trivia.samiControl:createController',
            'q_test = sami_trivia.questiontesting:main',
            'get_question = sami_trivia.get_question:main',
            'get_question_test_client = sami_trivia.get_question_test_client:main',            
        ],
    },
)
