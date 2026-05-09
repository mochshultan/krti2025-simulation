from setuptools import setup, Extension
import pybind11

ext_modules = [
    Extension(
        'binding',
        sources=['binding.cpp', 'cppmult.cpp'],
        include_dirs=[pybind11.get_include()],
        language='c++',
        extra_compile_args=['-O3', '-Wall', '-std=c++11']
    ),
]

setup(
    name='binding',
    version='0.1',
    ext_modules=ext_modules,
)
