set(srcs
        packet.cpp
        packet.h
        databuffer.h
        databuffer.cpp
        endianess.cpp
        cl_endian.h
        dataprocessor.cpp
        dataprocessor.h)

add_library(dataprocessor ${srcs})