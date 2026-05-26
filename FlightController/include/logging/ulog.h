#pragma once
#include <stdint.h>
#include <string.h>

#include <SdFat.h>
#include <SPI.h>
#include <Arduino.h>

#include "messages.h"
#include <string>


class Logger{

  private:
  
    bool m_initialized;
    String log_fname;
    FsFile log_file;
    SdFat sd;

    template<typename T> bool write(const T v);
    template<typename T> bool write_buffer(const T* buf, size_t len);

    bool write_file_header(const uint64_t timestamp_us);
    bool write_flag_bits();

  public: 

    Logger(){};
    ~Logger();
    Logger(uint8_t cs_pin, arduino::HardwareSPI& SPI);

    bool write_format_message(const ulog_view_t& view);
    bool write_subscription(const ulog_view_t& view);

    bool ulog_init(const uint64_t timestamp_us);
    bool write_data_message(const ulog_view_t& view);
    void flush();
};