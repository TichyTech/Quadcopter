#include "logging/ulog.h"

template<typename T>
bool Logger::write(const T v) {
  if (!m_initialized) {
    Serial.println("[Logger]: Attempted to write before initialization!");
    return false;
  }
  return log_file.write((const uint8_t*)&v, sizeof(T));
}

template<typename T>
bool Logger::write_buffer(const T* buf, size_t len) {
  if (!m_initialized) {
    Serial.println("[Logger]: Attempted to write before initialization!");
    return false;
  }
  return log_file.write((const uint8_t*)buf, len);
}

bool Logger::write_data_message(const ulog_view_t& view) {
  bool success = true;
  success &= write(view.header);
  success &= write(view.msg_id);
  success &= write_buffer(view.payload, view.payload_size);
  return success;
}

bool Logger::write_format_message(const ulog_view_t& view) {
    const char* topic_name = view.topic_name;
    const uint16_t topic_name_len = (uint16_t) strlen(topic_name);
    const char* fmt = view.fmt;
    const uint16_t fmt_len = (uint16_t) strlen(fmt);
    const message_header_t header = {fmt_len + topic_name_len, 'F'};
    return write(header) && write_buffer(topic_name, topic_name_len) && write_buffer(fmt, fmt_len);
}

bool Logger::write_subscription(const ulog_view_t& view) {
    const char* topic_name = view.topic_name;
    const uint16_t msg_id = view.msg_id;
    uint16_t name_len = strlen(topic_name);
    uint16_t msg_size = 1 + 2 + name_len;  // multi_id + msg_id + name
    const message_header_t header = {msg_size, 'A'};
    const uint8_t multi_id = 0;
    bool success = true;
    success &= write(header);
    success &= write(multi_id);
    success &= write(msg_id);
    success &= write_buffer(topic_name, name_len);
    return success;
}

bool Logger::write_file_header(const uint64_t timestamp_us) {
    const uint8_t magic[7] = { 0x55, 0x4c, 0x6f, 0x67, 0x01, 0x12, 0x35};
    const uint8_t version = 0x01;
    return write_buffer(magic, 7) && write(version) && write(timestamp_us);
}

bool Logger::write_flag_bits() {
    const message_header_t header = {40, 'B'};
    const uint8_t zeros[40] = {};
    return write(header) && write_buffer(zeros, 40);
}

bool Logger::ulog_init(const uint64_t timestamp_us) {
    bool success = true;
    success &= write_file_header(timestamp_us);
    success &= write_flag_bits();
    return success;
}

void Logger::flush() {
  if (m_initialized) {
    log_file.flush();
  }
}

Logger::Logger(uint8_t cs_pin, arduino::HardwareSPI &SPI)
{

  if (!sd.begin(SdSpiConfig(cs_pin, DEDICATED_SPI, SD_SCK_MHZ(50), &SPI))) {
    Serial.println("[Logger]: SD card failed to initialize!");
    return;
  }
  Serial.println("[Logger]: SD card initialized!");

  File32 root = sd.open("/");
  uint16_t num_files = 0;
  while (true) {
      File32 entry = root.openNextFile();
      if (!entry) break;  // no more files
      num_files += 1;
      Serial.print("File: ");
      entry.printName();
      Serial.print(" size: ");
      Serial.println(entry.size());
      entry.close();
  }

  log_fname = "log_" + String(num_files) + ".ulg";
  Serial.print("[Logger]: Starting to log into ");
  Serial.println(log_fname);
  log_file = sd.open(log_fname, FILE_WRITE);

  m_initialized = true;
  ulog_init(micros());

  Serial.println("[Logger]: Logging initialized successfully!");
}

Logger::~Logger() {
  if (log_file) {
    Serial.print("[Logger]: Closing log file: ");
    Serial.println(log_fname);
    log_file.close();
  }
};