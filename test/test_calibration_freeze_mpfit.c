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
#include <string.h>
#include "redist/os_generic.h"

// Exercise the real result handler with synthetic optimizer output and no device drivers.
#include "poser_mpfit.c"  // NOLINT(build/include): exercise upstream static handlers.

static unsigned lighthouse_updates;

static void accept_lighthouse(
  SurviveObject *so, uint8_t lighthouse, SurvivePose *pose,
  SurvivePose *object_pose, void *user)
{
  (void)object_pose;
  (void)user;
  ++lighthouse_updates;
  so->ctx->bsd[lighthouse].Pose = *pose;
}

int main(void)
{
  char *args[] = {"calibration-freeze-mpfit-test", "--configfile", "/dev/null", "--no-save-config"};
  SurviveContext *ctx = survive_init_internal(4, args, NULL, NULL);
  assert(ctx && ctx->driver_ct == 0);
  ctx->buttonQueue.buttonservicesem = OGCreateSema();
  SurviveObject object = {.ctx = ctx};
  MPFITData data = {.opt = {.so = &object}};
  struct async_optimizer_user user = {
    .d = &data, .worldEstablished = true, .canPossiblySolveLHS = true,
    .pdl = {.hdr = {.lighthouseposeproc = accept_lighthouse}}};
  user.meas_for_lhs_axis[0] = 1;
  user.meas_for_lhs_axis[1] = 1;
  SurvivePose parameters[2] = {
    {.Pos = {1, 2, 3}, .Rot = {1, 0, 0, 0}},
    {.Pos = {-4, 0, 0}, .Rot = {1, 0, 0, 0}}
  };
  survive_optimizer_parameter blocks[2] = {
    {.size = 7, .param_type = survive_optimizer_parameter_object_pose},
    {.size = 7, .param_type = survive_optimizer_parameter_camera}
  };
  survive_optimizer optimizer = {
    .poseLength = 1, .cameraLength = 1, .parameters = (FLT *)parameters,
    .parameters_info = blocks, .parameterBlockCnt = 2, .parametersCnt = 14,
    .stats = {.sensor_error_cnt = 1}};
  mp_result result = {.status = 1, .bestnorm = 0.25};
  SurvivePose output = {0};
  assert(handle_optimizer_results(&optimizer, 1, &result, &user, NULL, &output) == 0.25);
  assert(lighthouse_updates == 1);
  assert(ctx->bsd[0].Pose.Pos[0] == 4);
  assert(output.Pos[0] == 1);

  ctx->calibration_frozen = true;
  parameters[0].Pos[0] = 8;
  parameters[1].Pos[0] = -9;
  output.Pos[0] = 7;
  assert(handle_optimizer_results(&optimizer, 1, &result, &user, NULL, &output) < 0);
  assert(lighthouse_updates == 1 && "A stale calibration result changed a frozen Lighthouse pose");
  assert(ctx->bsd[0].Pose.Pos[0] == 4);
  assert(output.Pos[0] == 7 && "A stale calibration result changed the tracking pose");

  user.canPossiblySolveLHS = false;
  assert(handle_optimizer_results(&optimizer, 1, &result, &user, NULL, &output) == 0.25);
  assert(output.Pos[0] == 8 && "Freezing calibration also stopped ordinary tracking results");
  assert(lighthouse_updates == 1);
  assert(ctx->bsd[0].Pose.Pos[0] == 4);
  survive_close(ctx);
  return 0;
}
