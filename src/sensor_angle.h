#ifndef __SENSOR_ANGLE_H
#define __SENSOR_ANGLE_H

#include <stdint.h> // uint8_t

struct spi_angle;
struct spi_angle *spi_angle_oid_lookup(uint8_t oid);
int spi_angle_get_latest(struct spi_angle *sa, uint32_t *time, uint32_t *angle);
int spi_angle_get_angle_bits(struct spi_angle *sa);

#endif // sensor_angle.h
