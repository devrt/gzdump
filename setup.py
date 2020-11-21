import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

if __name__ == "__main__":
    setuptools.setup(
        name='gzdump',
        version='0.0.1',
        author="Yosuke Matsusaka",
        author_email="yosuke.matsusaka@gmail.com",
        description="Utility command to dump pose of objects in gazebo",
        long_description=long_description,
        long_description_content_type="text/markdown",
        url="https://github.com/devrt/gzdump",
        packages=setuptools.find_packages(),
        classifiers=[
            "Programming Language :: Python :: 2",
            "License :: OSI Approved :: MIT License",
            "Operating System :: OS Independent",
        ],
        entry_points={
            'console_scripts': [
                'gzdump = gzdump.dump:main',
                'gzundump = gzdump.undump:main'
            ]
        }
    )
