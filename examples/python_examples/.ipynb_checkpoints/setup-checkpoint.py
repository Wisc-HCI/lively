from setuptools import setup
from setuptools_rust import Binding, RustExtension
from pathlib import Path

this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

package_name = 'lively_tk'

setup(
    name=package_name,
    version='0.9.8',
    packages=[package_name],
    rust_extensions=[RustExtension("lively_tk.lively_tk", binding=Binding.PyO3, quiet=True, features=['pybindings'])],
    install_requires=['setuptools','wheel','setuptools_rust'],
    zip_safe=False,
    maintainer='AndrewJSchoen',
    maintainer_email='schoen.andrewj@gmail.com',
    description='A real-time robot motion control framework with built-in liveliness',
    long_description=long_description,
    long_description_content_type='text/markdown',
    license='MIT License (MIT)',
    tests_require=['pytest']
)
