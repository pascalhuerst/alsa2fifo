cmake_minimum_required(VERSION 3.2)

project(alsa2fifo)

include(FindPkgConfig)

function(addLib name)
  pkg_check_modules(${name} REQUIRED ${name})
  include_directories(${${name}_INCLUDE_DIRS})
  link_directories(${${name}_LIBRARY_DIRS})
endfunction(addLib)

function(linkLib name target)
  pkg_check_modules(${name} REQUIRED ${name})
  target_link_libraries(${target} ${${name}_LIBRARIES})
endfunction(linkLib)

set(LIBS alsa avahi-client avahi-core)
set(SOURCES alsa2fifo.cpp AlsaAudioInput.cpp publishZeroConf/publishAvahi.cpp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wno-unused-parameter -Wall -O2")

add_executable(${PROJECT_NAME} ${SOURCES})

foreach(lib ${LIBS})
  addLib(${lib})
endforeach(lib)

foreach(lib ${LIBS})
	linkLib(${lib} ${PROJECT_NAME})
endforeach(lib)

find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})


install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION /usr/sbin COMPONENT runtime)
