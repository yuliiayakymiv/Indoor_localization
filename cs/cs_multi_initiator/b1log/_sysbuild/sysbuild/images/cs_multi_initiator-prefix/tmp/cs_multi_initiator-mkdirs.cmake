# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "E:/UCU/2_course/1_semester/POK/project/Indoor_localization/cs/cs_multi_initiator")
  file(MAKE_DIRECTORY "E:/UCU/2_course/1_semester/POK/project/Indoor_localization/cs/cs_multi_initiator")
endif()
file(MAKE_DIRECTORY
  "E:/UCU/2_course/1_semester/POK/project/Indoor_localization/cs/cs_multi_initiator/b1log/cs_multi_initiator"
  "E:/UCU/2_course/1_semester/POK/project/Indoor_localization/cs/cs_multi_initiator/b1log/_sysbuild/sysbuild/images/cs_multi_initiator-prefix"
  "E:/UCU/2_course/1_semester/POK/project/Indoor_localization/cs/cs_multi_initiator/b1log/_sysbuild/sysbuild/images/cs_multi_initiator-prefix/tmp"
  "E:/UCU/2_course/1_semester/POK/project/Indoor_localization/cs/cs_multi_initiator/b1log/_sysbuild/sysbuild/images/cs_multi_initiator-prefix/src/cs_multi_initiator-stamp"
  "E:/UCU/2_course/1_semester/POK/project/Indoor_localization/cs/cs_multi_initiator/b1log/_sysbuild/sysbuild/images/cs_multi_initiator-prefix/src"
  "E:/UCU/2_course/1_semester/POK/project/Indoor_localization/cs/cs_multi_initiator/b1log/_sysbuild/sysbuild/images/cs_multi_initiator-prefix/src/cs_multi_initiator-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/UCU/2_course/1_semester/POK/project/Indoor_localization/cs/cs_multi_initiator/b1log/_sysbuild/sysbuild/images/cs_multi_initiator-prefix/src/cs_multi_initiator-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "E:/UCU/2_course/1_semester/POK/project/Indoor_localization/cs/cs_multi_initiator/b1log/_sysbuild/sysbuild/images/cs_multi_initiator-prefix/src/cs_multi_initiator-stamp${cfgdir}") # cfgdir has leading slash
endif()
