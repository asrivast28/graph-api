# Graph API
This repository provides a uniform API for different graph implementations.

## Requirements
* **gcc** (with C++11 support)  
This project has been test built only on Linux platform, using gcc with C++11 support.
* **SCons**  
SCons, a cross-platform Python based build environment, is required for building the project.
 More information about the tool can be found at [http://scons.org/](http://scons.org/).
* **Boost Graph Library**  
[Boost Graph Library](http://www.boost.org/doc/libs/1_58_0/libs/graph/doc/index.html) is a header-only library that contains different graph implementations. Currently, only `boost::adjacency_list` is supported by the API but it can be extended to support other graph types.

## Building
This repository is not intended to be used in a stand-alone mode. Therefore, only an `SConscript` file is provided for the repository, which can be
called by the `SConstruct` of another project.
