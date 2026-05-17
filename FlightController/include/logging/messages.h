#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>

#define ULOG_MSG_STRUCT(MSG_TYPE_NAME) \
    struct MSG_TYPE_NAME : ulog_msg_t<MSG_TYPE_NAME<Name, TopicId>>


struct message_header_t {
  uint16_t msg_size;
  uint8_t msg_type;
} __attribute__((packed));


struct ulog_view_t {
    const message_header_t header;
    const uint16_t msg_id;
    const uint8_t* payload;
    uint16_t       payload_size;
    const char*    fmt;
    const char* topic_name;
};


template<typename Derived>
struct ulog_msg_t {
    ulog_view_t view() const {
        const Derived& d = static_cast<const Derived&>(*this);
        return 
        {
            .header = d.header,
            .msg_id = Derived::MsgId,
            .payload = reinterpret_cast<const uint8_t*>(&d.payload),
            .payload_size  = sizeof(d.payload),
            .fmt   = d.fmt,
            .topic_name = d.topic_name
        };
    }
};

template<const char* Name, uint16_t TopicId>
ULOG_MSG_STRUCT(ulog_float_msg_t) {
    union {
        struct {
            uint64_t timestamp;
            float    value;
        } __attribute__((packed)) data;
        uint8_t bytes[sizeof(data)];
    } payload;

    const message_header_t header = { .msg_size = sizeof(payload) + 2, .msg_type = 'D' };
    constexpr static uint16_t MsgId = TopicId;
    static constexpr const char* topic_name = Name;
    static constexpr const char* fmt = ":uint64_t timestamp;float value;";

    static_assert(sizeof(payload) + 2 == 14, "ulog_float_msg_t: payload size mismatch — check struct packing");
};

template<const char* Name, uint16_t TopicId>
ULOG_MSG_STRUCT(ulog_int_msg_t) {
    union {
        struct {
            uint64_t timestamp;
            int32_t  value;
        } __attribute__((packed)) data;
        uint8_t bytes[sizeof(data)];
    } payload;

    const message_header_t header = { .msg_size = sizeof(payload) + 2, .msg_type = 'D' };
    constexpr static uint16_t MsgId = TopicId;
    static constexpr const char* topic_name = Name;
    static constexpr const char* fmt = ":uint64_t timestamp;int32_t value;";

    static_assert(sizeof(payload) + 2 == 14, "ulog_int_msg_t: payload size mismatch — check struct packing");
};

template<const char* Name, uint16_t TopicId>
ULOG_MSG_STRUCT(ulog_vector3f_msg_t) {
    
    union {
        struct {
            uint64_t timestamp;
            float  x;
            float  y;
            float  z;
        } __attribute__((packed)) data;
        uint8_t bytes[sizeof(data)];
    } payload;

    const message_header_t header = { .msg_size = sizeof(payload) + 2, .msg_type = 'D' };
    constexpr static uint16_t MsgId = TopicId;
    static constexpr const char* topic_name = Name;
    static constexpr const char* fmt = ":uint64_t timestamp;float x;float y;float z;";

    static_assert(sizeof(payload) + 2 == 22, "ulog_vector3f_msg_t: payload size mismatch — check struct packing");
};

#endif // MESSAGES_H