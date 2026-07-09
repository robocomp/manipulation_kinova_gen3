# Sources set
LIST(APPEND SOURCES
        ../src/specificworker.cpp
        ../src/api_kinova_controller.cpp
)

# Headers set
LIST(APPEND HEADERS
        ../src/specificworker.h
        ../src/api_kinova_controller.h
        KinovaArm.h
)

SET (LIBS ${LIBS})

add_compile_options(
    -O3 
    -march=native 
    -flto=auto
    -fno-plt
)
add_definitions(-DNDEBUG -fmax-errors=1)
add_link_options(-flto=auto)
link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/../lib/release/libKortexApiCpp.a)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include/client)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include/common)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include/messages)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include/client_stubs)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cxxopts)



