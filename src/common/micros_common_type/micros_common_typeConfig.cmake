# Config for micros_common_type (header-only)
# Install layout: share/micros_common_type/cmake/ -> package prefix is ../../..
get_filename_component(micros_common_type_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(micros_common_type_PREFIX "${micros_common_type_DIR}/../../.." ABSOLUTE)
set(micros_common_type_INCLUDE_DIRS "${micros_common_type_PREFIX}/include")
