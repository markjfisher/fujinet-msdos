/**
 * FujiNet NIO low-level protocol helpers.
 */

#include "nio.h"
#include "portio.h"
#include <string.h>

#define NIO_TIMEOUT_SLOW (15 * 1000)
#define NIO_MAX_RX       9216
#define NIO_MAX_TX_PREFIX 32

enum {
  SLIP_END     = 0xC0,
  SLIP_ESCAPE  = 0xDB,
  SLIP_ESC_END = 0xDC,
  SLIP_ESC_ESC = 0xDD,
};

enum {
  FUJI_FIELD_NONE = 0,
  FUJI_FIELD_A1   = 1,
};

typedef struct {
  uint8_t device;
  uint8_t command;
  uint16_t length;
  uint8_t checksum;
  uint8_t fields;
} nio_header_t;

static uint8_t tx_prefix[NIO_MAX_TX_PREFIX];
static uint8_t rx_payload[NIO_MAX_RX];
static nio_header_t rx_header;
uint8_t nio_last_error;
uint8_t nio_last_status;
uint16_t nio_last_rx_len;
uint16_t nio_last_expected_len;
uint8_t nio_last_lsr;
uint16_t nio_network_timeout_ms = NIO_TIMEOUT_SLOW;

static bool nio_fail(uint8_t error, uint16_t rx_len, uint16_t expected_len)
{
  nio_last_error = error;
  nio_last_rx_len = rx_len;
  nio_last_expected_len = expected_len;
  return false;
}

static uint8_t nio_port_error(uint8_t fallback)
{
  if (port_slip_last_lsr)
    return NIO_ERR_UART;
  switch (port_slip_last_reason) {
  case PORT_SLIP_REASON_TIMEOUT:
    return NIO_ERR_TIMEOUT;
  case PORT_SLIP_REASON_BUFFER_FULL:
    return NIO_ERR_BUFFER_FULL;
  case PORT_SLIP_REASON_LINE_STATUS:
    return NIO_ERR_UART;
  default:
    return fallback;
  }
}

static void put_u16le(uint8_t far *p, uint16_t v)
{
  p[0] = (uint8_t) (v & 0xFF);
  p[1] = (uint8_t) ((v >> 8) & 0xFF);
}

static void put_u32le(uint8_t far *p, uint32_t v)
{
  p[0] = (uint8_t) (v & 0xFF);
  p[1] = (uint8_t) ((v >> 8) & 0xFF);
  p[2] = (uint8_t) ((v >> 16) & 0xFF);
  p[3] = (uint8_t) ((v >> 24) & 0xFF);
}

static uint16_t get_u16le(const uint8_t far *p)
{
  return (uint16_t) p[0] | ((uint16_t) p[1] << 8);
}

static uint32_t get_u32le(const uint8_t far *p)
{
  return (uint32_t) p[0]
      | ((uint32_t) p[1] << 8)
      | ((uint32_t) p[2] << 16)
      | ((uint32_t) p[3] << 24);
}

static uint16_t nio_calc_checksum(const void far *ptr, uint16_t len, uint16_t seed)
{
  uint16_t idx, chk;
  const uint8_t far *buf = (const uint8_t far *) ptr;

  for (idx = 0, chk = seed; idx < len; idx++)
    chk = ((chk + buf[idx]) >> 8) + ((chk + buf[idx]) & 0xFF);
  return chk;
}

bool nio_status_ok(uint8_t status)
{
  return status == NIO_STATUS_OK;
}

bool nio_call(uint8_t device, uint8_t command,
              const void far *payload, uint16_t payload_length,
              void far *reply, uint16_t reply_capacity,
              nio_response_t far *response)
{
  nio_header_t *tx = (nio_header_t *) tx_prefix;
  uint16_t checksum;
  uint16_t rx_len;
  uint16_t rx_payload_len;
  uint16_t payload_offset;
  uint16_t timeout;
  uint8_t status;

  if (response) {
    response->status = NIO_STATUS_INTERNAL_ERROR;
    response->payload_length = 0;
  }
  nio_last_error = NIO_ERR_NONE;
  nio_last_status = NIO_STATUS_INTERNAL_ERROR;
  nio_last_rx_len = 0;
  nio_last_expected_len = 0;
  nio_last_lsr = 0;

  tx->device = device;
  tx->command = command;
  tx->length = sizeof(*tx) + payload_length;
  tx->checksum = 0;
  tx->fields = FUJI_FIELD_NONE;

  checksum = nio_calc_checksum(tx, sizeof(*tx), 0);
  if (payload)
    checksum = nio_calc_checksum(payload, payload_length, checksum);
  tx->checksum = (uint8_t) checksum;

  port_flush_rx();
  port_putc(SLIP_END);
  port_putbuf_slip(tx_prefix, sizeof(*tx));
  if (payload && payload_length)
    port_putbuf_slip(payload, payload_length);
  port_putc(SLIP_END);
  port_wait_tx_empty();

  timeout = (device == NIO_DEVICEID_NETWORK) ? nio_network_timeout_ms : NIO_TIMEOUT_SLOW;
  rx_len = port_getbuf_slip_dual(&rx_header, sizeof(rx_header),
                                 rx_payload, sizeof(rx_payload),
                                 timeout);
  nio_last_rx_len = rx_len;
  nio_last_lsr = port_slip_last_lsr;
  if (rx_len < sizeof(rx_header)) {
    nio_last_expected_len = sizeof(rx_header);
    return nio_fail(nio_port_error(NIO_ERR_SHORT_FRAME), rx_len, sizeof(rx_header));
  }
  nio_last_expected_len = rx_header.length;
  if (rx_len != rx_header.length)
    return nio_fail(nio_port_error(NIO_ERR_LENGTH_MISMATCH), rx_len, rx_header.length);

  rx_payload_len = rx_len - sizeof(rx_header);

  checksum = rx_header.checksum;
  rx_header.checksum = 0;
  if ((uint8_t) nio_calc_checksum(rx_payload, rx_payload_len,
        nio_calc_checksum(&rx_header, sizeof(rx_header), 0)) != checksum)
    return nio_fail(NIO_ERR_CHECKSUM, rx_len, rx_header.length);

  if (rx_header.device != device || rx_header.command != command)
    return nio_fail(NIO_ERR_DEVICE_COMMAND, rx_len, rx_header.length);

  if ((rx_header.fields & 0x80) || (rx_header.fields & 0x07) != FUJI_FIELD_A1)
    return nio_fail(NIO_ERR_FIELDS, rx_len, rx_header.length);
  if (rx_payload_len < 1)
    return nio_fail(NIO_ERR_EMPTY_STATUS, rx_len, rx_header.length);

  status = rx_payload[0];
  nio_last_status = status;
  payload_offset = 1;
  rx_payload_len -= payload_offset;

  if (rx_payload_len > reply_capacity)
    return nio_fail(NIO_ERR_REPLY_TOO_LARGE, rx_len, rx_header.length);
  if (reply && rx_payload_len)
    _fmemmove(reply, rx_payload + payload_offset, rx_payload_len);

  if (response) {
    response->status = status;
    response->payload_length = rx_payload_len;
  }

  return true;
}

bool nio_disk_info(uint8_t slot, nio_disk_info_t far *info)
{
  uint8_t req[2];
  uint8_t resp[16];
  nio_response_t nr;

  req[0] = NIO_DISK_VERSION;
  req[1] = slot;

  if (!nio_call(NIO_DEVICEID_DISK, NIO_DISK_CMD_INFO,
                req, sizeof(req), resp, sizeof(resp), &nr))
    return false;
  if (!nio_status_ok(nr.status) || nr.payload_length < 12 || resp[0] != NIO_DISK_VERSION)
    return nio_fail(nio_status_ok(nr.status) ? NIO_ERR_PAYLOAD : NIO_ERR_STATUS,
                    nio_last_rx_len, nio_last_expected_len);

  info->flags = resp[1];
  info->slot = resp[4];
  info->image_type = resp[5];
  info->sector_size = get_u16le(&resp[6]);
  info->sector_count = get_u32le(&resp[8]);
  info->last_error = (nr.payload_length > 12) ? resp[12] : 0;
  return true;
}

bool nio_disk_read_sector(uint8_t slot, uint32_t lba,
                          void far *buffer, uint16_t buffer_length,
                          uint16_t far *bytes_read)
{
  uint8_t req[8];
  nio_response_t nr;
  uint16_t data_len;

  req[0] = NIO_DISK_VERSION;
  req[1] = slot;
  put_u32le(&req[2], lba);
  put_u16le(&req[6], buffer_length);

  if (!nio_call(NIO_DEVICEID_DISK, NIO_DISK_CMD_READ_SECTOR,
                req, sizeof(req), rx_payload, sizeof(rx_payload), &nr))
    return false;
  if (!nio_status_ok(nr.status) || nr.payload_length < 11 || rx_payload[0] != NIO_DISK_VERSION)
    return nio_fail(nio_status_ok(nr.status) ? NIO_ERR_PAYLOAD : NIO_ERR_STATUS,
                    nio_last_rx_len, nio_last_expected_len);

  data_len = get_u16le(&rx_payload[9]);
  if (data_len > buffer_length || nr.payload_length < (uint16_t) (11 + data_len))
    return nio_fail(NIO_ERR_PAYLOAD, nio_last_rx_len, nio_last_expected_len);

  if (buffer && data_len)
    _fmemcpy(buffer, rx_payload + 11, data_len);
  if (bytes_read)
    *bytes_read = data_len;
  return true;
}

bool nio_disk_read_sectors(uint8_t slot, uint32_t lba, uint16_t count,
                           void far *buffer, uint16_t buffer_length,
                           uint16_t far *bytes_read)
{
  uint8_t req[10];
  nio_response_t nr;
  uint16_t data_len;

  req[0] = NIO_DISK_VERSION;
  req[1] = slot;
  put_u32le(&req[2], lba);
  put_u16le(&req[6], count);
  put_u16le(&req[8], buffer_length);

  if (!nio_call(NIO_DEVICEID_DISK, NIO_DISK_CMD_READ_SECTORS,
                req, sizeof(req), rx_payload, sizeof(rx_payload), &nr))
    return false;
  if (!nio_status_ok(nr.status) || nr.payload_length < 13 || rx_payload[0] != NIO_DISK_VERSION)
    return nio_fail(nio_status_ok(nr.status) ? NIO_ERR_PAYLOAD : NIO_ERR_STATUS,
                    nio_last_rx_len, nio_last_expected_len);

  data_len = get_u16le(&rx_payload[11]);
  if (data_len > buffer_length || nr.payload_length < (uint16_t) (13 + data_len))
    return nio_fail(NIO_ERR_PAYLOAD, nio_last_rx_len, nio_last_expected_len);

  if (buffer && data_len)
    _fmemcpy(buffer, rx_payload + 13, data_len);
  if (bytes_read)
    *bytes_read = data_len;
  return true;
}

bool nio_disk_write_sector(uint8_t slot, uint32_t lba,
                           const void far *buffer, uint16_t buffer_length,
                           uint16_t far *bytes_written)
{
  uint8_t req_prefix[8];
  uint8_t resp[16];
  nio_response_t nr;

  req_prefix[0] = NIO_DISK_VERSION;
  req_prefix[1] = slot;
  put_u32le(&req_prefix[2], lba);
  put_u16le(&req_prefix[6], buffer_length);

  /* Build and send this request manually so sector data does not need to be
     copied into a second 16-bit DOS buffer. */
  {
    nio_header_t *tx = (nio_header_t *) tx_prefix;
    uint16_t checksum;
    uint16_t rx_len;
    uint16_t rx_payload_len;

    tx->device = NIO_DEVICEID_DISK;
    tx->command = NIO_DISK_CMD_WRITE_SECTOR;
    tx->length = sizeof(*tx) + sizeof(req_prefix) + buffer_length;
    tx->checksum = 0;
    tx->fields = FUJI_FIELD_NONE;

    checksum = nio_calc_checksum(tx, sizeof(*tx), 0);
    checksum = nio_calc_checksum(req_prefix, sizeof(req_prefix), checksum);
    checksum = nio_calc_checksum(buffer, buffer_length, checksum);
    tx->checksum = (uint8_t) checksum;

    port_flush_rx();
    port_putc(SLIP_END);
    port_putbuf_slip(tx_prefix, sizeof(*tx));
    port_putbuf_slip(req_prefix, sizeof(req_prefix));
    if (buffer && buffer_length)
      port_putbuf_slip(buffer, buffer_length);
    port_putc(SLIP_END);
    port_wait_tx_empty();

    rx_len = port_getbuf_slip_dual(&rx_header, sizeof(rx_header),
                                   resp, sizeof(resp), NIO_TIMEOUT_SLOW);
    if (rx_len < sizeof(rx_header) || rx_len != rx_header.length)
      return false;
    rx_payload_len = rx_len - sizeof(rx_header);
    checksum = rx_header.checksum;
    rx_header.checksum = 0;
    if ((uint8_t) nio_calc_checksum(resp, rx_payload_len,
          nio_calc_checksum(&rx_header, sizeof(rx_header), 0)) != checksum)
      return false;
    if (rx_header.device != NIO_DEVICEID_DISK || rx_header.command != NIO_DISK_CMD_WRITE_SECTOR)
      return false;
    if ((rx_header.fields & 0x80) || (rx_header.fields & 0x07) != FUJI_FIELD_A1 || rx_payload_len < 1)
      return false;

    nr.status = resp[0];
    nr.payload_length = rx_payload_len - 1;
    if (!nio_status_ok(nr.status) || nr.payload_length < 11 || resp[1] != NIO_DISK_VERSION)
      return false;
    if (bytes_written)
      *bytes_written = get_u16le(&resp[10]);
  }

  return true;
}

bool nio_disk_write_sectors(uint8_t slot, uint32_t lba, uint16_t count,
                            const void far *buffer, uint16_t buffer_length,
                            uint16_t far *bytes_written)
{
  uint8_t req_prefix[10];
  uint8_t resp[16];
  nio_response_t nr;

  req_prefix[0] = NIO_DISK_VERSION;
  req_prefix[1] = slot;
  put_u32le(&req_prefix[2], lba);
  put_u16le(&req_prefix[6], count);
  put_u16le(&req_prefix[8], buffer_length);

  {
    nio_header_t *tx = (nio_header_t *) tx_prefix;
    uint16_t checksum;
    uint16_t rx_len;
    uint16_t rx_payload_len;

    tx->device = NIO_DEVICEID_DISK;
    tx->command = NIO_DISK_CMD_WRITE_SECTORS;
    tx->length = sizeof(*tx) + sizeof(req_prefix) + buffer_length;
    tx->checksum = 0;
    tx->fields = FUJI_FIELD_NONE;

    checksum = nio_calc_checksum(tx, sizeof(*tx), 0);
    checksum = nio_calc_checksum(req_prefix, sizeof(req_prefix), checksum);
    checksum = nio_calc_checksum(buffer, buffer_length, checksum);
    tx->checksum = (uint8_t) checksum;

    port_flush_rx();
    port_putc(SLIP_END);
    port_putbuf_slip(tx_prefix, sizeof(*tx));
    port_putbuf_slip(req_prefix, sizeof(req_prefix));
    if (buffer && buffer_length)
      port_putbuf_slip(buffer, buffer_length);
    port_putc(SLIP_END);
    port_wait_tx_empty();

    rx_len = port_getbuf_slip_dual(&rx_header, sizeof(rx_header),
                                   resp, sizeof(resp), NIO_TIMEOUT_SLOW);
    if (rx_len < sizeof(rx_header) || rx_len != rx_header.length)
      return false;
    rx_payload_len = rx_len - sizeof(rx_header);
    checksum = rx_header.checksum;
    rx_header.checksum = 0;
    if ((uint8_t) nio_calc_checksum(resp, rx_payload_len,
          nio_calc_checksum(&rx_header, sizeof(rx_header), 0)) != checksum)
      return false;
    if (rx_header.device != NIO_DEVICEID_DISK || rx_header.command != NIO_DISK_CMD_WRITE_SECTORS)
      return false;
    if ((rx_header.fields & 0x80) || (rx_header.fields & 0x07) != FUJI_FIELD_A1 || rx_payload_len < 1)
      return false;

    nr.status = resp[0];
    nr.payload_length = rx_payload_len - 1;
    if (!nio_status_ok(nr.status) || nr.payload_length < 13 || resp[1] != NIO_DISK_VERSION)
      return false;
    if (bytes_written)
      *bytes_written = get_u16le(&resp[12]);
  }

  return true;
}
