cmake_minimum_required(VERSION 3.5)
find_package(pybind11 QUIET)
add_compile_options(-std=c++14)

message(STATUS "check for pybind11")

if(${pybind11_FOUND})

    message(STATUS "compiling python bindings")

    execute_process(
      COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
        from distutils import sysconfig as sc
        print(sc.get_python_lib(prefix='', plat_specific=True))"
      OUTPUT_VARIABLE PYTHON_SITE
      OUTPUT_STRIP_TRAILING_WHITESPACE)

    if(DEFINED ENV{CONDA_PREFIX}) # if using a conda env, install bindings there
      set(PYBIND_INSTALL_DIR $ENV{CONDA_PREFIX}/${PYTHON_SITE}/${PROJECT_NAME} CACHE PATH "bindings install prefix" FORCE)
    else()
      set(PYBIND_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE}/${PROJECT_NAME} CACHE PATH "bindings install prefix" FORCE)
    endif()

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(pyphase pyPhase.cpp)
    target_link_libraries(pyphase PUBLIC phase_manager)

    pybind11_add_module(pymanager pyPhaseManager.cpp)
    target_link_libraries(pymanager PUBLIC phase_manager)

    pybind11_add_module(pytimeline pyTimeline.cpp)
    target_link_libraries(pytimeline PUBLIC phase_manager)
    
    install(TARGETS pytimeline
            DESTINATION ${PYBIND_INSTALL_DIR})
    install(TARGETS pymanager
            DESTINATION ${PYBIND_INSTALL_DIR})
    install(TARGETS pyphase
            DESTINATION ${PYBIND_INSTALL_DIR})

    if(WITH_ROS_COMPONENTS)
      
      pybind11_add_module(pyrosserver pyRosServer.cpp)
      target_link_libraries(pyrosserver PUBLIC phase_manager)

      install(TARGETS pyrosserver
              DESTINATION ${PYBIND_INSTALL_DIR})

    endif()

    install(FILES __init__.py DESTINATION ${PYBIND_INSTALL_DIR})    

else()
    message(STATUS "pybind not found")
endif()
