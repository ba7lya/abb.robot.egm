# SPDX-License-Identifier: MIT

# The library's own C++ standard is PRIVATE, so every test target sets its own
# requirement instead of inheriting one from the library.

add_subdirectory(ut)
add_subdirectory(integration)
