cmake_minimum_required(VERSION 2.8.3)
project(rqt_volturnus)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rqt_gui rqt_gui_cpp std_msgs)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)

include(${QT_USE_FILE})

set(rqt_volturnus_SRCS
  src/volturnus_gui.cpp
)

set(rqt_volturnus_HDRS
  src/volturnus_gui.h
)

set(rqt_volturnus_UIS
  src/volturnus_gui.ui
)

set(rqt_volturnus_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
	INCLUDE_DIRS ${rqt_volturnus_INCLUDE_DIRECTORIES}
	LIBRARIES ${PROJECT_NAME} 
        CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp std_msgs
)
catkin_python_setup()

qt4_wrap_cpp(rqt_volturnus_MOCS ${rqt_volturnus_HDRS})
qt4_wrap_ui(rqt_volturnus_UIS_H ${rqt_volturnus_UIS})

include_directories(${rqt_volturnus_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_volturnus_SRCS} ${rqt_volturnus_MOCS} ${rqt_volturnus_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

#add_executable(volturnus_gui ${rqt_volturnus_SRCS})
#target_link_libraries(volturnus_gui ${catkin_LIBRARIES})
#add_dependencies(volturnus_gui ${catkin_EXPORTED_TARGETS})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(PROGRAMS scripts/rqt_volturnus
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
