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
#include <dlfcn.h>
#include "redist/os_generic.h"
#include "libsurvive/survive.h"
#include "src/survive_internal.h"

int main(int argc, char **argv)
{
  assert(argc == 2);
  void *plugin = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
  assert(plugin && "The built calibration plugin could not be loaded");
  DeviceDriver register_solver = (DeviceDriver)GetDriver("DriverRegGlobalSceneSolver");
  assert(register_solver && GetDriver("FreezeGlobalSceneSolver"));
  char *args[] = {"calibration-plugin-test", "--configfile", "/dev/null", "--no-save-config"};
  SurviveContext *ctx = survive_init_internal(4, args, NULL, NULL);
  assert(ctx && ctx->driver_ct == 0);
  ctx->buttonQueue.buttonservicesem = OGCreateSema();
  ctx->activeLighthouses = 1;
  ctx->bsd[0].PositionSet = true;
  ctx->bsd[0].OOTXSet = true;
  ctx->bsd[0].BaseStationID = 0x1234;
  ctx->bsd[0].Pose.Rot[0] = 1;
  assert(register_solver(ctx) == SURVIVE_DRIVER_PASSIVE);
  survive_release_ctx_lock(ctx);
  assert(survive_freeze_calibration(ctx));
  assert(ctx->calibration_frozen);
  assert(survive_freeze_calibration(ctx));
  survive_close(ctx);
  dlclose(plugin);
  return 0;
}
