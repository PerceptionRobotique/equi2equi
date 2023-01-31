cmake_minimum_required(VERSION 3.0)

PROJECT(equi2equi)

FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
	INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)

FIND_PACKAGE(Boost REQUIRED filesystem regex)

# libPeR prEqui related functionalities...
find_package(PER REQUIRED per_core per_io)
if(PER_FOUND)
	include(${PER_USE_FILE})
endif(PER_FOUND)

ADD_EXECUTABLE(equi2equi

		equi2equi.cpp
)

MESSAGE(${PER_LIBRARIES})

target_link_libraries(equi2equi ${Boost_LIBRARIES})
