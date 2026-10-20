set(GLS_TEST_SOURCES
  test_main.cpp
  test_foundations.cpp
  test_fracture.cpp
)
add_executable(gls_tests ${GLS_TEST_SOURCES})
target_link_libraries(gls_tests PRIVATE gls_core)
target_compile_options(gls_tests PRIVATE ${GLS_WARNINGS})

# The runner prints per-case results; pass a substring to run a subset: gls_tests "plate:"
add_test(NAME gls_tests COMMAND gls_tests)
