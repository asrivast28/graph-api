# Graph API
This repository provides a uniform API for different graph implementations.

## Requirements
* **gcc** (with C++11 support)  
This project has been test built only on Linux platform, using gcc with C++11 support.
* **Boost Graph Library**  
[Boost Graph Library](http://www.boost.org/doc/libs/1_58_0/libs/graph/doc/index.html) is a header-only library that contains different graph implementations. Currently, only `boost::adjacency_list` is supported by the API but it can be extended to support other graph types.

## Building
This is a header-only library, intended to be used by including the relevant header files.
