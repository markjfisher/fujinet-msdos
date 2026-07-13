#ifndef FUJI_IOCTL_H
#define FUJI_IOCTL_H

#include <stdint.h>
#include <nio.h>

#define FUJI_IOCTL_SIGNATURE "FUJI"
#define FUJI_IOCTL_VERSION   1
#define FUJI_IOCTL_MAX_DATA  512
#define FUJI_IOCTL_MAX_URI   255
#define FUJI_IOCTL_MAX_PATH  127
#define FUJI_IOCTL_NIO_DIAG_MAX_RECORDS 8

enum {
  FUJI_IOCTL_QUERY = 0,
  FUJI_IOCTL_GET_STATE = 1,
  FUJI_IOCTL_SET_STATE = 2,
  FUJI_IOCTL_GET_UNIT_MAP = 3,
  FUJI_IOCTL_SET_UNIT_MAP = 4,
  FUJI_IOCTL_NIO_CALL = 5,
  FUJI_IOCTL_NIO_DIAG = 6
};

typedef struct {
  uint8_t command;
  char signature[4];
  uint8_t unit;
  uint8_t version;
  uint8_t max_units;
} fuji_ioctl_query;

typedef struct {
  uint8_t command;
  char signature[4];
  uint8_t unit;
  uint8_t version;
  uint8_t max_units;
  uint16_t current_uri_len;
  uint16_t display_path_len;
  char current_uri[FUJI_IOCTL_MAX_URI + 1];
  char display_path[FUJI_IOCTL_MAX_PATH + 1];
} fuji_ioctl_state;

typedef struct {
  uint8_t command;
  char signature[4];
  uint8_t unit;
  uint8_t version;
  uint8_t max_units;
  uint8_t slot;
} fuji_ioctl_unit_map;

typedef struct {
  uint8_t command;
  char signature[4];
  uint8_t unit;
  uint8_t version;
  uint8_t device;
  uint8_t nio_command;
  uint8_t nio_status;
  uint16_t request_len;
  uint16_t response_len;
  uint8_t data[FUJI_IOCTL_MAX_DATA];
  uint8_t diag_error;
  uint8_t diag_status;
  uint16_t diag_rx_len;
  uint16_t diag_expected_len;
  uint8_t diag_lsr;
} fuji_ioctl_nio_call;

typedef struct {
  uint8_t command;
  char signature[4];
  uint8_t unit;
  uint8_t version;
  uint8_t clear;
  uint16_t start;
  uint16_t max_records;
  uint16_t record_size;
  uint16_t record_count;
  uint16_t available;
  uint32_t total;
  uint32_t dropped;
  nio_diag_record_t records[FUJI_IOCTL_NIO_DIAG_MAX_RECORDS];
} fuji_ioctl_nio_diag;

#endif /* FUJI_IOCTL_H */
