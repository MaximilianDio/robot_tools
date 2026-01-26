from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name='robot_tools',
    version='0.1.0',
    author='Your Name',
    description='A Python package for working with robot models and visualization using URDF files and PyVista',
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=find_packages(exclude=['tests', 'tests.*', 'examples', 'examples.*', 'build', 'build.*']) + ['urdfpy'],
    package_dir={
        'urdfpy': 'robot_visualization/urdfpy/urdfpy'
    },
    python_requires='>=3.7',
    install_requires=[
        'numpy',
        'pyvista',
        'scipy',
        # urdfpy dependencies (since we're including it as a submodule)
        'lxml',
        'networkx>=3.0',
        'pillow',
        'pycollada==0.6',
        'pyrender>=0.1.20',
        'six',
        'trimesh',
    ],
    extras_require={
        'dev': [
            'pytest',
            'pytest-cov',
            'flake8',
        ],
    },
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'Topic :: Scientific/Engineering :: Visualization',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
    ],
)