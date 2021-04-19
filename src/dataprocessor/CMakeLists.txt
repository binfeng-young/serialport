set(srcs
        packet.cpp
        packet.h
        databuffer.h
        databuffer.cpp
        endianess.cpp
        cl_endian.h
        dataprocessor.cpp
        dataprocessor.h
        crc.h
        crc.cpp)

add_library(dataprocessor ${srcs})