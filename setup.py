from setuptools import setup
from setuptools_rust import Binding, RustExtension

package_name = 'lively_ik_core'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    rust_extensions=[RustExtension("lively_ik_core.lively_ik_core", binding=Binding.PyO3, quiet=True)],
    install_requires=['setuptools','wheel','setuptools_rust'],
    zip_safe=False,
    maintainer='AndrewJSchoen',
    maintainer_email='schoen.andrewj@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest']
)
