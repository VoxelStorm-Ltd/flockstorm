# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/runner/work/flockstorm/flockstorm/build/_deps/vectorstorm-src")
  file(MAKE_DIRECTORY "/home/runner/work/flockstorm/flockstorm/build/_deps/vectorstorm-src")
endif()
file(MAKE_DIRECTORY
  "/home/runner/work/flockstorm/flockstorm/build/_deps/vectorstorm-build"
  "/home/runner/work/flockstorm/flockstorm/build/_deps/vectorstorm-subbuild/vectorstorm-populate-prefix"
  "/home/runner/work/flockstorm/flockstorm/build/_deps/vectorstorm-subbuild/vectorstorm-populate-prefix/tmp"
  "/home/runner/work/flockstorm/flockstorm/build/_deps/vectorstorm-subbuild/vectorstorm-populate-prefix/src/vectorstorm-populate-stamp"
  "/home/runner/work/flockstorm/flockstorm/build/_deps/vectorstorm-subbuild/vectorstorm-populate-prefix/src"
  "/home/runner/work/flockstorm/flockstorm/build/_deps/vectorstorm-subbuild/vectorstorm-populate-prefix/src/vectorstorm-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/runner/work/flockstorm/flockstorm/build/_deps/vectorstorm-subbuild/vectorstorm-populate-prefix/src/vectorstorm-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/runner/work/flockstorm/flockstorm/build/_deps/vectorstorm-subbuild/vectorstorm-populate-prefix/src/vectorstorm-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
