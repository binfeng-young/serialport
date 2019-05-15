set(exec_name ${PROJECT_NAME})

#set(CMAKE_PREFIX_PATH "/opt/Qt5.6.1/5.6/gcc_64")
find_package(Qt5 REQUIRED COMPONENTS Widgets SerialPort)
set(QT_LIBRARIES Qt5::Widgets Qt5::SerialPort)

include_directories(dataprocessor)
add_subdirectory(dataprocessor)
include_directories(mapping)

file(GLOB_RECURSE ALL_LIBRARY_HDRS "./mapping/*.h")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "./mapping/*.cpp")

set(baseic_srcs
    main.cpp
    #mainwindow.cpp
    serialportwidget.cpp
    serialportthread.cpp
    map_thread.cpp
    )
set(moc_files
    #mainwindow.h
    serialportwidget.h
    serialportthread.h
    map_thread.h
    )
set(uic_files
    #mainwindow.ui
    serialportwidget.ui
    )
set(qrc_files ${RESOURCES_PATH}/serialport.qrc)

QT5_WRAP_CPP(moc_srcs ${moc_files})
QT5_WRAP_UI(uic_srcs ${uic_files})
QT5_ADD_RESOURCES(qrc_srcs ${qrc_files})

set(srcs
    ${baseic_srcs}
    ${moc_srcs}
    ${uic_srcs}
    ${qrc_srcs})
add_executable(${exec_name} ${srcs} ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})
#set_target_properties(${exec_name}  PROPERTIES ENVIRONMENT "${PROJECT_DEPENDS_PATH}")
target_link_libraries(${exec_name} ${QT_LIBRARIES} dataprocessor)