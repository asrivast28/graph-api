##
# @file SConscript
# @brief Provides functionality for building the graph library.

import platform

Import('env')

libName = 'graph'

srcFiles = [
            'GraphFile.cpp',
            'Edge.cpp',
            'Vertex.cpp',
            'Graph.cpp',
            ]

graph = env.StaticLibrary(target = libName, source = srcFiles)
#graph = env.SharedLibrary(target = libName, source = srcFiles)
Return('graph')
