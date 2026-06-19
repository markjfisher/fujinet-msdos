#include "commands.h"
#include "fujinet.h"
#include "sys_hdr.h"
#include "nio.h"
#include "print.h"
#include "ioctl.h"
#include <string.h>
#include <dos.h>

#undef DEBUG

#define SECTOR_SIZE     512

extern void End_code(void);

DOS_BPB fn_bpb_table[FN_MAX_DEV];
DOS_BPB *fn_bpb_pointers[FN_MAX_DEV + 1]; // leave room for the NULL terminator

static uint8_t nio_unit_slot[FN_MAX_DEV] = {0, 1, 2, 3, 4, 5, 6, 7};
static char nio_current_uri[FUJI_IOCTL_MAX_URI + 1];
static char nio_display_path[FUJI_IOCTL_MAX_PATH + 1];
static uint16_t nio_current_uri_len;
static uint16_t nio_display_path_len;

#ifdef OBSOLETE
static cmdFrame_t cmd; // FIXME - make this shared with init.c?
#endif /* OBSOLETE */

static uint8_t unit_to_slot(uint8_t unit)
{
  if (unit >= FN_MAX_DEV)
    return 0xFF;
  return nio_unit_slot[unit];
}

static uint8_t unit_to_diskservice_slot(uint8_t unit)
{
  uint8_t slot = unit_to_slot(unit);
  if (slot == 0xFF)
    return 0xFF;
  return slot + 1;
}

static void fill_query(fuji_ioctl_query far *query, uint8_t unit)
{
  _fmemcpy(query->signature, FUJI_IOCTL_SIGNATURE, 4);
  query->unit = unit;
  query->version = FUJI_IOCTL_VERSION;
  query->max_units = FN_MAX_DEV;
}

static uint16_t handle_ioctl_buffer(SYSREQ far *req)
{
  uint8_t far *buffer = req->io.buffer_ptr;
  uint8_t command;

  if (req->unit >= FN_MAX_DEV) {
    consolef("Invalid IOCTL unit: %i\n", req->unit);
    return ERROR_BIT | UNKNOWN_UNIT;
  }

  if (req->io.count < sizeof(fuji_ioctl_query))
    return ERROR_BIT | UNKNOWN_CMD;

  command = buffer[0];

  if (command == FUJI_IOCTL_QUERY) {
    fill_query((fuji_ioctl_query far *) buffer, req->unit);
    return OP_COMPLETE;
  }

  if (req->io.count < 5)
    return ERROR_BIT | UNKNOWN_CMD;

  switch (command) {
  case FUJI_IOCTL_GET_STATE:
  {
    fuji_ioctl_state far *state = (fuji_ioctl_state far *) buffer;
    if (req->io.count < sizeof(*state))
      return ERROR_BIT | UNKNOWN_CMD;

    fill_query((fuji_ioctl_query far *) state, req->unit);
    state->current_uri_len = nio_current_uri_len;
    state->display_path_len = nio_display_path_len;
    _fmemcpy(state->current_uri, nio_current_uri, sizeof(nio_current_uri));
    _fmemcpy(state->display_path, nio_display_path, sizeof(nio_display_path));
    return OP_COMPLETE;
  }

  case FUJI_IOCTL_SET_STATE:
  {
    fuji_ioctl_state far *state = (fuji_ioctl_state far *) buffer;
    if (req->io.count < sizeof(*state))
      return ERROR_BIT | UNKNOWN_CMD;
    if (state->current_uri_len > FUJI_IOCTL_MAX_URI ||
        state->display_path_len > FUJI_IOCTL_MAX_PATH)
      return ERROR_BIT | BAD_REQ_LEN;

    nio_current_uri_len = state->current_uri_len;
    nio_display_path_len = state->display_path_len;
    _fmemset(nio_current_uri, 0, sizeof(nio_current_uri));
    _fmemset(nio_display_path, 0, sizeof(nio_display_path));
    _fmemcpy(nio_current_uri, state->current_uri, nio_current_uri_len);
    _fmemcpy(nio_display_path, state->display_path, nio_display_path_len);
    fill_query((fuji_ioctl_query far *) state, req->unit);
    return OP_COMPLETE;
  }

  case FUJI_IOCTL_GET_UNIT_MAP:
  {
    fuji_ioctl_unit_map far *map = (fuji_ioctl_unit_map far *) buffer;
    if (req->io.count < sizeof(*map))
      return ERROR_BIT | UNKNOWN_CMD;
    if (map->unit >= FN_MAX_DEV)
      return ERROR_BIT | UNKNOWN_UNIT;

    fill_query((fuji_ioctl_query far *) map, req->unit);
    map->slot = nio_unit_slot[map->unit];
    return OP_COMPLETE;
  }

  case FUJI_IOCTL_SET_UNIT_MAP:
  {
    fuji_ioctl_unit_map far *map = (fuji_ioctl_unit_map far *) buffer;
    if (req->io.count < sizeof(*map))
      return ERROR_BIT | UNKNOWN_CMD;
    if (map->unit >= FN_MAX_DEV)
      return ERROR_BIT | UNKNOWN_UNIT;
    if (map->slot >= FN_MAX_DEV)
      return ERROR_BIT | BAD_REQ_LEN;

    nio_unit_slot[map->unit] = map->slot;
    fill_query((fuji_ioctl_query far *) map, req->unit);
    return OP_COMPLETE;
  }

  case FUJI_IOCTL_NIO_CALL:
  {
    fuji_ioctl_nio_call far *call = (fuji_ioctl_nio_call far *) buffer;
    nio_response_t response;

    if (req->io.count < sizeof(*call))
      return ERROR_BIT | UNKNOWN_CMD;
    if (call->request_len > FUJI_IOCTL_MAX_DATA ||
        call->response_len > FUJI_IOCTL_MAX_DATA)
      return ERROR_BIT | BAD_REQ_LEN;

    if (!nio_call(call->device, call->nio_command,
                  call->data, call->request_len,
                  call->data, call->response_len,
                  &response))
      return ERROR_BIT | GENERAL_FAIL;

    call->nio_status = response.status;
    call->response_len = response.payload_length;
    fill_query((fuji_ioctl_query far *) call, req->unit);
    return OP_COMPLETE;
  }

  default:
    return ERROR_BIT | UNKNOWN_CMD;
  }
}

uint16_t Media_check_cmd(SYSREQ far *req)
{
  nio_disk_info_t info;

  int i=0;

  // Avoid race condition that only happens on PCjr systems
  // I do not know why this works. -Thom
  for (i=0;i<8192;i++);

  if (req->unit >= FN_MAX_DEV) {
    consolef("Invalid Media Check unit: %i\n", req->unit);
    return ERROR_BIT | UNKNOWN_UNIT;
  }

  if (!nio_disk_info(unit_to_diskservice_slot(req->unit), &info))
    return ERROR_BIT | NOT_READY;

  if (!(info.flags & NIO_DISK_INFO_INSERTED))
    req->media.return_info = 0;
  else if (info.flags & NIO_DISK_INFO_CHANGED)
    req->media.return_info = -1;
  else
    req->media.return_info = 1;

  return OP_COMPLETE;
}

uint16_t Build_bpb_cmd(SYSREQ far *req)
{
  uint8_t far *buf;
  uint16_t bytes_read;


  if (req->unit >= FN_MAX_DEV) {
    consolef("Invalid BPB unit: %i\n", req->unit);
    return ERROR_BIT | UNKNOWN_UNIT;
  }

  // DOS gave us a buffer to use
  buf = req->bpb.buffer_ptr;
  if (!nio_disk_read_sector(unit_to_diskservice_slot(req->unit), 0, buf, SECTOR_SIZE, &bytes_read)
      || bytes_read < SECTOR_SIZE) {
    consolef("FujiNet NIO BPB read fail\n");
    return ERROR_BIT | READ_FAULT;
  }

  _fmemcpy(fn_bpb_pointers[req->unit], &buf[0x0b], sizeof(DOS_BPB));

#if 0
  consolef("BPB for %i\n", req->unit);
  dumpHex((uint8_t far *) fn_bpb_pointers[req->unit], sizeof(DOS_BPB));
#endif

  req->bpb.table = MK_FP(getCS(), fn_bpb_pointers[req->unit]);

  return OP_COMPLETE;
}

uint16_t Ioctl_input_cmd(SYSREQ far *req)
{
#if 0
  consolef("IOCTL INPUT CALLED\n");
  consolef("UNIT: %d\n", req->unit);
  consolef("SIZE: %d\n", req->io.count);
  consolef("BUFFER: %08lx\n", req->io.buffer_ptr);
#endif

  if (req->unit >= FN_MAX_DEV) {
    consolef("Invalid Input unit: %i\n", req->unit);
    return ERROR_BIT | UNKNOWN_UNIT;
  }

  return handle_ioctl_buffer(req);
}

uint16_t Input_cmd(SYSREQ far *req)
{
  uint16_t idx;
  uint32_t sector, sector_max;
  uint8_t far *buf = req->io.buffer_ptr;
  uint16_t bytes_read;


  if (req->unit >= FN_MAX_DEV) {
    consolef("Invalid Input unit: %i\n", req->unit);
    return ERROR_BIT | UNKNOWN_UNIT;
  }

#if 0
  dumpHex((uint8_t far *) req, req->length);
  consolef("SECTOR: %i 0x%04x 0x%08lx\n", req->length,
           req->io.start_sector, (uint32_t) req->io.start_sector_32);
#endif

  if (req->length > 22)
    sector = req->io.start_sector_32;
  else
    sector = req->io.start_sector;

  if (fn_bpb_table[req->unit].num_sectors)
    sector_max = fn_bpb_table[req->unit].num_sectors;
  else
    sector_max = fn_bpb_table[req->unit].num_sectors_32;

#if 0
  consolef("SECTOR: %i 0x%08lx %i SM: %i\n", req->length, sector, req->io.count, sector_max);
#endif

  for (idx = 0; idx < req->io.count; idx++, sector++) {
    if (sector >= sector_max) {
      consolef("FN Invalid sector read %li on %i\n", sector, req->unit);
      return ERROR_BIT | NOT_FOUND;
    }

    if (!nio_disk_read_sector(unit_to_diskservice_slot(req->unit), sector,
                              &buf[idx * SECTOR_SIZE], SECTOR_SIZE,
                              &bytes_read) || bytes_read < SECTOR_SIZE)
      break;
  }
  if (!idx)
    return ERROR_BIT | GENERAL_FAIL;

  req->io.count = idx;
  return OP_COMPLETE;
}

uint16_t Input_no_wait_cmd(SYSREQ far *req)
{
  return UNKNOWN_CMD;
}

uint16_t Input_status_cmd(SYSREQ far *req)
{
  return UNKNOWN_CMD;
}

uint16_t Input_flush_cmd(SYSREQ far *req)
{
  consolef("FN INPUT FLUSH\n");
  return ERROR_BIT | GENERAL_FAIL;
}

uint16_t Output_cmd(SYSREQ far *req)
{
  uint16_t idx;
  uint32_t sector, sector_max;
  uint8_t far *buf = req->io.buffer_ptr;
  uint16_t bytes_written;
  nio_disk_info_t info;


  if (req->unit >= FN_MAX_DEV) {
    consolef("Invalid Output unit: %i\n", req->unit);
    return ERROR_BIT | UNKNOWN_UNIT;
  }

  if (!nio_disk_info(unit_to_diskservice_slot(req->unit), &info))
    return ERROR_BIT | NOT_READY;

  if (info.flags & NIO_DISK_INFO_READONLY)
    return ERROR_BIT | WRITE_PROTECT;

  if (req->length > 22)
    sector = req->io.start_sector_32;
  else
    sector = req->io.start_sector;

  if (fn_bpb_table[req->unit].num_sectors)
    sector_max = fn_bpb_table[req->unit].num_sectors;
  else
    sector_max = fn_bpb_table[req->unit].num_sectors_32;

#if 0
  consolef("WRITE SECTOR: %i 0x%08lx %i\n", req->length, sector, req->io.count);
#endif

  for (idx = 0; idx < req->io.count; idx++, sector++) {
    if (sector >= sector_max) {
      consolef("FN Invalid sector write %i on %i:\n", sector, req->unit);
      return ERROR_BIT | NOT_FOUND;
    }

    if (!nio_disk_write_sector(unit_to_diskservice_slot(req->unit), sector,
                               &buf[idx * SECTOR_SIZE], SECTOR_SIZE,
                               &bytes_written) || bytes_written < SECTOR_SIZE)
      break;
  }
  if (!idx)
    return ERROR_BIT | GENERAL_FAIL;

  req->io.count = idx;
  return OP_COMPLETE;
}

uint16_t Output_verify_cmd(SYSREQ far *req)
{
  consolef("FN OUTPUT VERIFY\n");
  req->io.count = 0;
  return ERROR_BIT | GENERAL_FAIL;
}

uint16_t Output_status_cmd(SYSREQ far *req)
{
  return UNKNOWN_CMD;
}

uint16_t Output_flush_cmd(SYSREQ far *req)
{
  return UNKNOWN_CMD;
}

uint16_t Ioctl_output_cmd(SYSREQ far *req)
{
  return handle_ioctl_buffer(req);
}

uint16_t Dev_open_cmd(SYSREQ far *req)
{
  consolef("FN DEV OPEN\n");
  return ERROR_BIT | GENERAL_FAIL;
}

uint16_t Dev_close_cmd(SYSREQ far *req)
{
  return UNKNOWN_CMD;
}

uint16_t Remove_media_cmd(SYSREQ far *req)
{
  return UNKNOWN_CMD;
}

uint16_t Ioctl_cmd(SYSREQ far *req)
{
  consolef("IOCTL CALLED\n");
  return UNKNOWN_CMD;
}

uint16_t Get_l_d_map_cmd(SYSREQ far *req)
{
  consolef("FN GET LD MAP\n");
  req->ldmap.unit_code = 0;
  return ERROR_BIT | GENERAL_FAIL;
}

uint16_t Set_l_d_map_cmd(SYSREQ far *req)
{
  consolef("FN SET LD MAP\n");
  req->ldmap.unit_code = 0;
  return ERROR_BIT | GENERAL_FAIL;
}

uint16_t Unknown_cmd(SYSREQ far *req)
{
  return UNKNOWN_CMD;
}
