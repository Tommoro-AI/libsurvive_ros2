// Copyright 2026 Sunghyun Kwon
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <zlib.h>

#include "redist/os_generic.h"
#include "libsurvive/survive.h"
#include "src/survive_default_devices.h"

static void input_bit(SurviveObject *so, int station, int version, int bit)
{
  survive_default_sync_process(so, (survive_channel)station,
    so->last_sync_time[station] + 958000, bit != 0, version != 0);
}

static size_t packet_bits(int *bits, int version, uint32_t id, int8_t up, bool corrupt)
{
  const uint16_t length = version ? 43 : 33;
  uint8_t bytes[50] = {0};
  memcpy(bytes, &length, sizeof(length));
  memcpy(bytes + 4, &id, sizeof(id));
  bytes[22] = (uint8_t)up;
  uint32_t crc = (uint32_t)crc32(0, bytes + 2, length);
  if (corrupt) {
    crc ^= 1;
  }
  const size_t padded = length + (length & 1);
  memcpy(bytes + 2 + padded, &crc, sizeof(crc));
  size_t count = 0;
  for (int i = 0; i < 17; ++i) {
    bits[count++] = 0;
  }
  bits[count++] = 1;
  for (size_t i = 0; i < padded + 6; ++i) {
    for (int bit = 7; bit >= 0; --bit) {
      bits[count++] = (bytes[i] >> bit) & 1;
    }
    if (i % 2 == 1) {
      bits[count++] = 1;
    }
  }
  return count;
}

static void feed(SurviveObject *so, int station, int version, const int *bits, size_t count)
{
  for (size_t i = 0; i < count; ++i) {
    input_bit(so, (int8_t)station, (int8_t)version, bits[i]);
  }
}

static void test_version(int version)
{
  char *args[] = {"ootx-test", "--configfile", "/dev/null", "--no-save-config",
    "--threaded-posers", "0"};
  SurviveContext *ctx = survive_init_internal(6, args, NULL, NULL);
  assert(ctx && ctx->driver_ct == 0);
  ctx->buttonQueue.buttonservicesem = OGCreateSema();
  ctx->lh_version = version;
  ctx->activeLighthouses = 2;
  for (int i = 0; i < 2; ++i) {
    ctx->bsd_map[i] = (int8_t)i;
    ctx->bsd[i].mode = (uint8_t)i;
  }
  SurviveObject *first = survive_create_device(ctx, "test", NULL, "WM0", NULL);
  SurviveObject *second = survive_create_device(ctx, "test", NULL, "WM1", NULL);
  survive_add_object(ctx, first);
  survive_add_object(ctx, second);
  int valid[512], invalid[512];
  size_t count = packet_bits(valid, version, 0x12345678, 50, false);
  packet_bits(invalid, version, 0x87654321, 90, true);

  input_bit(first, 0, (int8_t)version, 0);
  feed(second, 0, version, valid, count);
  assert(ctx->bsd[0].OOTXSet && ctx->bsd[0].BaseStationID == 0x12345678);
  assert(!ctx->bsd[0].PositionSet && !ctx->bsd[1].OOTXSet);

  feed(first, 1, version, invalid, count);
  assert(!ctx->bsd[1].OOTXSet && !ctx->bsd[1].PositionSet);

  for (size_t i = 0; i < count; ++i) {
    input_bit(first, 1, (int8_t)version, invalid[i]);
    input_bit(second, 1, (int8_t)version, valid[i]);
  }
  assert(ctx->bsd[1].OOTXSet && ctx->bsd[1].BaseStationID == 0x12345678);
  ctx->bsd[1].PositionSet = true;
  feed(first, 1, version, invalid, count);
  assert(ctx->bsd[1].BaseStationID == 0x12345678 && ctx->bsd[1].PositionSet);

  feed(second, 1, version, valid, count);
  assert(ctx->bsd[1].PositionSet);
  packet_bits(valid, version, 0x12345678, 60, false);
  feed(second, 1, version, valid, count);
  assert(!ctx->bsd[1].PositionSet && ctx->bsd[1].accel[0] == 60);

  survive_destroy_device(second);
  packet_bits(valid, version, 0xabcdef12, 70, false);
  feed(first, 1, version, valid, count);
  assert(ctx->bsd[1].BaseStationID == 0xabcdef12);
  survive_ootx_free_decoder_context(ctx, 0);
  assert(ctx->bsd[0].ootx_data == NULL);
  survive_ootx_free_decoder_context(ctx, 0);
  survive_destroy_device(first);
  assert(ctx->bsd[1].ootx_data == NULL);
  survive_close(ctx);
}

int main(void)
{
  test_version(0);
  test_version(1);
  return 0;
}
