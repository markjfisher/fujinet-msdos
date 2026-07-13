#include "../sys/ioctl.h"
#include <ctype.h>
#include <i86.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FUJI_SIGNATURE "FUJI"

static int ioctl_read(int drive, void *buffer, unsigned size)
{
  union REGS regs;
  struct SREGS sregs;

  regs.h.ah = 0x44;
  regs.h.al = 0x04;
  regs.h.bl = (unsigned char) drive;
  regs.w.cx = size;
  regs.x.dx = FP_OFF(buffer);
  sregs.ds = FP_SEG(buffer);

  int86x(0x21, &regs, &regs, &sregs);
  return (regs.x.cflag & INTR_CF) ? 0 : 1;
}

static int find_driver_drive(void)
{
  int drive;
  fuji_ioctl_query query;

  for (drive = 3; drive <= 26; drive++) {
    memset(&query, 0, sizeof(query));
    query.command = FUJI_IOCTL_QUERY;
    if (ioctl_read(drive, &query, sizeof(query)) &&
        memcmp(query.signature, FUJI_SIGNATURE, 4) == 0)
      return drive;
  }
  return -1;
}

static void print_prefix(FILE *out, const unsigned char *prefix)
{
  int i;
  for (i = 0; i < NIO_DIAG_REQ_PREFIX; i++)
    fprintf(out, "%s%02X", i ? " " : "", prefix[i]);
}

static void write_record(FILE *out, const nio_diag_record_t *rec)
{
  fprintf(out,
          "seq=%lu tick=%lu event=%u attempt=%u/%u dev=%02X cmd=%02X "
          "err=%u st=%u rx=%u exp=%u lsr=%02X reason=%u "
          "req=%u cap=%u timeout=%u tx=%u pre=%02X post=%02X prefix=",
          (unsigned long) rec->seq,
          (unsigned long) rec->tick,
          rec->event,
          rec->attempt,
          rec->max_attempts,
          rec->device,
          rec->command,
          rec->error,
          rec->status,
          rec->rx_len,
          rec->expected_len,
          rec->lsr,
          rec->slip_reason,
          rec->request_len,
          rec->reply_capacity,
          rec->timeout_ms,
          rec->tx_encoded_len,
          rec->pre_flush_lsr,
          rec->post_tx_lsr);
  print_prefix(out, rec->request_prefix);
  fputc('\n', out);
}

static int dump_log(int drive, const char *path, int clear_after)
{
  FILE *out;
  fuji_ioctl_nio_diag diag;
  unsigned start = 0;
  unsigned idx;
  int wrote = 0;

  out = fopen(path, "wt");
  if (!out) {
    fprintf(stderr, "Unable to open %s\n", path);
    return 1;
  }

  for (;;) {
    memset(&diag, 0, sizeof(diag));
    diag.command = FUJI_IOCTL_NIO_DIAG;
    diag.start = start;
    diag.max_records = FUJI_IOCTL_NIO_DIAG_MAX_RECORDS;

    if (!ioctl_read(drive, &diag, sizeof(diag)) ||
        memcmp(diag.signature, FUJI_SIGNATURE, 4) != 0) {
      fclose(out);
      fprintf(stderr, "Unable to read NIO diagnostics from driver\n");
      return 1;
    }

    if (start == 0) {
      fprintf(out,
              "NIO diagnostics: available=%u total=%lu dropped=%lu record_size=%u\n",
              diag.available,
              (unsigned long) diag.total,
              (unsigned long) diag.dropped,
              diag.record_size);
    }

    if (diag.record_count == 0)
      break;

    for (idx = 0; idx < diag.record_count; idx++) {
      write_record(out, &diag.records[idx]);
      wrote++;
    }
    start += diag.record_count;
  }

  fclose(out);

  if (clear_after) {
    memset(&diag, 0, sizeof(diag));
    diag.command = FUJI_IOCTL_NIO_DIAG;
    diag.clear = 1;
    ioctl_read(drive, &diag, sizeof(diag));
  }

  printf("Wrote %u record(s) to %s\n", wrote, path);
  return 0;
}

int main(int argc, char **argv)
{
  const char *path = "C:\\NIOLOG.TXT";
  int clear_after = 0;
  int drive;
  int i;

  for (i = 1; i < argc; i++) {
    if (argv[i][0] == '-' || argv[i][0] == '/') {
      int opt = toupper(argv[i][1]);
      if (opt == 'C')
        clear_after = 1;
      else {
        fprintf(stderr, "Usage: %s [outfile] [-c]\n", argv[0]);
        return 1;
      }
    } else {
      path = argv[i];
    }
  }

  drive = find_driver_drive();
  if (drive < 0) {
    fprintf(stderr, "FujiNet DOS driver not found\n");
    return 1;
  }

  return dump_log(drive, path, clear_after);
}
