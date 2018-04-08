set(exec_name ${PROJECT_NAME})

set(CMAKE_PREFIX_PATH "/opt/Qt5.6.1/5.6/gcc_64")
find_package(Qt5 REQUIRED COMPONENTS Widgets SerialPort)
set(QT_LIBRARIES Qt5::Widgets Qt5::SerialPort)

set(baseic_srcs
    main.cpp
    mainwindow.cpp
    serialportwidget.cpp
    serialportthread.cpp
    )
set(moc_files
    mainwindow.h
    serialportwidget.h
    serialportthread.h
    )
set(uic_files
    mainwindow.ui
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
add_executable(${exec_name} ${srcs})
#set_target_properties(${exec_name}  PROPERTIES ENVIRONMENT "${PROJECT_DEPENDS_PATH}")
target_link_libraries(${exec_name} ${QT_LIBRARIES})