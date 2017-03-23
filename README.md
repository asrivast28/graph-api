# Graph API
This repository provides a uniform API for different graph implementations.

## Requirements
* **gcc** (with C++11 support)  
This project has been test built only on Linux platform, using gcc with C++11 support.
* **SCons**  
SCons, a cross-platform Python based build environment, is required for building the project.
 More information about the tool can be found at [http://scons.org/](http://scons.org/).

## Building
This repository is not intended to be used in a stand-alone mode. Therefore, we only provide an `SConscript` file for the repository, which can be
called by the `SConstruct` of another project.
