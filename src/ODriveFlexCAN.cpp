#include "ODriveFlexCAN.h"

float CanbusConverters::buffer_to_float(const uint8_t *buffer)
{
    float y;
    uint32_t *const py = (uint32_t *)&y;

    *py = ((uint32_t)buffer[3] << 24) |
          ((uint32_t)buffer[2] << 16) |
          ((uint32_t)buffer[1] << 8) |
          ((uint32_t)buffer[0] << 0);

    return y;
}

uint32_t CanbusConverters::buffer_to_uint32(const uint8_t *buffer)
{
    return uint32_t((buffer[3] << 24) |
                    (buffer[2] << 16) |
                    (buffer[1] << 8) |
                    buffer[0]);
}

void CanbusConverters::uint32_to_buffer(uint32_t payload, uint8_t *buffer)
{
    buffer[3] = (payload >> 24) & 0xFF;
    buffer[2] = (payload >> 16) & 0xFF;
    buffer[1] = (payload >> 8) & 0xFF;
    buffer[0] = payload & 0xFF;
}

void CanbusConverters::float_to_buffer(float payload, uint8_t *buffer)
{
    uint8_t *f_byte = reinterpret_cast<uint8_t *>(&payload);
    memcpy(buffer, f_byte, 4);
}