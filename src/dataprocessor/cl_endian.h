//
// Created by binfeng.yang on 2018/12/10.
//
#pragma once


/// \addtogroup bvCore_I_O_Data bvCore I/O Data
/// \{

/// \brief Endianess management class.
class Endian
{
public:
    /// \brief Swaps larger amounts of data between little and big endian.
    ///
    /// \param data Data to be swapped.
    /// \param type_size Size of datatype to be swapped. (Must be an even number)
    /// \param total_times Number of 'type_size' size data chunks to be swapped.
    static void swap(void *data, int type_size, int total_times = 1);

    static void swap_if_big(void *data, int type_size, int total_times = 1) { if (is_system_big()) swap(data, type_size, total_times); }
    static void swap_if_little(void *data, int type_size, int total_times = 1) { if (!is_system_big()) swap(data, type_size, total_times); }

    /// \brief Returns true if big endian system.
    /** \return True if big endian system, false otherwise.*/
    static bool is_system_big();

    /// \brief Returns true if 64 bit system.
    /** \return True if 64 bit, false otherwise.*/
    static bool is_system_64bit();
};

/// \}
