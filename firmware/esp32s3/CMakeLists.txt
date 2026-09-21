# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# The wire contract is shared with the STM32 node, so its directory is
# registered as a component named shared. This has to come before the include
# below, the boilerplate order matters.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../shared")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(BloomcareGateway)
