cmake_minimum_required(VERSION 3.20)
project(GlassDynamics VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(GLS_BUILD_TESTS "Build Glass Dynamics tests" ON)
option(GLS_BUILD_TOOLS "Build the glassdemo tool" ON)
option(GLS_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

if(MSVC)
  set(GLS_WARNINGS /W4 /permissive-)
  if(GLS_WARNINGS_AS_ERRORS)
    list(APPEND GLS_WARNINGS /WX)
  endif()
else()
  set(GLS_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
  if(GLS_WARNINGS_AS_ERRORS)
    list(APPEND GLS_WARNINGS -Werror)
  endif()
endif()

# ---- Engine-independent glazing fracture core (no engine/renderer headers) ----
add_library(gls_core
  src/core/src/json.cpp
  src/core/src/library.cpp
  src/core/src/plate.cpp
  src/core/src/contact.cpp
  src/core/src/cracks.cpp
  src/core/src/fragments.cpp
  src/core/src/collapse.cpp
  src/core/src/pane.cpp
  src/core/src/bridge.cpp
  src/core/src/rupert.cpp
)
add_library(gls::core ALIAS gls_core)
target_include_directories(gls_core PUBLIC src/core/include)
target_compile_options(gls_core PRIVATE ${GLS_WARNINGS})
target_compile_definitions(gls_core PUBLIC GLS_DATA_DIR="${PROJECT_SOURCE_DIR}/data")

if(GLS_BUILD_TOOLS)
  add_executable(glassdemo tools/glassdemo/main.cpp)
  target_link_libraries(glassdemo PRIVATE gls_core)
  target_compile_options(glassdemo PRIVATE ${GLS_WARNINGS})
endif()

if(GLS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
