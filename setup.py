from setuptools import setup
from setuptools_rust import Binding, RustExtension

package_name = 'lively_tk'

setup(
    name=package_name,
    version='0.6.0',
    packages=[package_name],
    rust_extensions=[RustExtension("lively_tk.lively_tk", binding=Binding.PyO3, quiet=True)],
    install_requires=['setuptools','wheel','setuptools_rust'],
    zip_safe=False,
    maintainer='AndrewJSchoen',
    maintainer_email='schoen.andrewj@gmail.com',
    description='A real-time robot motion control framework',
    license='MIT License (MIT)',
    tests_require=['pytest']
)
