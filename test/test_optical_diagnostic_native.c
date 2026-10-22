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
#include "poser_mpfit.c"  // NOLINT(build/include): exercise upstream static handlers.
#include "survive_api.c"  // NOLINT(build/include): exercise upstream static handlers.

static void test_geometry(const survive_reproject_model_t *model, bool moving)
{
  char *args[] = {"optical-diagnostic-test", "--configfile", "/dev/null", "--no-save-config"};
  SurviveContext *ctx = survive_init_internal(4, args, NULL, NULL);
  assert(ctx && ctx->driver_ct == 0);
  ctx->buttonQueue.buttonservicesem = OGCreateSema();
  ctx->activeLighthouses = 2;
  FLT sensors[6] = {-.1, 0, 0, .1, .1, 0};
  SurviveOpticalDiagnostic diagnostic = {.connected = true};
  SurviveObject object = {.optical_diagnostic = &diagnostic, .ctx = ctx, .timebase_hz = 48000000,
    .sensor_ct = 2,
    .sensor_locations = sensors};
  SurviveObject *objects[] = {&object};
  MPFITData data = {.opt = {.so = &object}};
  struct async_optimizer_user user = {.d = &data, .worldEstablished = true,
    .pdl = {.hdr = {.timecode = 48000000}}};
  user.stats.time_window = 4800000;
  SurvivePose parameters[4] = {
    {.Pos = {0, 0, -2}, .Rot = {1, 0, 0, 0}},
    {.Rot = {1, 0, 0, 0}}, {.Pos = {1, 0, 0}, .Rot = {1, 0, 0, 0}}, {.Pos = {.5, 0, 0}}
  };
  survive_optimizer_parameter blocks[4] = {
    {.size = 7, .param_type = survive_optimizer_parameter_object_pose},
    {.size = 7, .param_type = survive_optimizer_parameter_camera},
    {.size = 7, .param_type = survive_optimizer_parameter_camera},
    {.size = 6, .param_type = survive_optimizer_parameter_object_velocity}
  };
  survive_optimizer_settings settings = {.use_quat_model = true, .disable_filter = true,
    .optimize_scale_threshold = -1};
  survive_optimizer_measurement measurements[8] = {0};
  survive_optimizer optimizer = {
    .settings = &settings, .reprojectModel = model,
    .sos = objects, .poseLength = 1, .cameraLength = 2, .disableVelocity = !moving,
    .parameters = (FLT *)parameters, .parameters_info = blocks,
    .parameterBlockCnt = moving ? 4 : 3, .parametersCnt = moving ? 27 : 21,
    .measurements = measurements, .measurementsCnt = 8, .timecode = 1
  };
  for (int lh = 0; lh < 2; ++lh) {
    for (int sensor = 0; sensor < 2; ++sensor) {
      LinmathPoint3d point, world;
      for (int axis = 0; axis < 2; ++axis) {
        int i = lh * 4 + sensor * 2 + axis;
        FLT time = .99 + i * .001;
        ApplyPoseToPoint(world, &parameters[0], sensors + 3 * sensor);
        if (moving) {world[0] -= .5 * (1 - time);}
        ApplyPoseToPoint(point, &parameters[lh + 1], world);
        FLT angles[2];
        optimizer.reprojectModel->reprojectXY(ctx->bsd[lh].fcal, point, angles);
        measurements[i] = (survive_optimizer_measurement) {
          .meas_type = survive_optimizer_measurement_type_light, .size = 1, .variance = 1,
          .time = time, .light = {.object = 0, .lh = lh, .sensor_idx = sensor,
            .axis = axis, .value = angles[axis]}
        };
      }
    }
  }
  assert(survive_optimizer_current_norm(&optimizer) < 1e-12);
  const SurvivePose before = parameters[0];
  record_optical_diagnostic(&optimizer, 1, &user, true);
  assert(object.optical_diagnostic->rms_rad < 1e-6);
  assert(object.optical_diagnostic->residual_count == 8);
  assert(object.optical_diagnostic->peak_lighthouse_count == 2);
  assert(fabs(object.optical_diagnostic->max_age_seconds - .01) < 1e-6);
  assert(memcmp(&before, &parameters[0], sizeof(before)) == 0);

  parameters[2].Pos[0] += .2;
  assert(survive_optimizer_current_norm(&optimizer) > .001);
  record_optical_diagnostic(&optimizer, -1, &user, true);
  const FLT peak = object.optical_diagnostic->peak_rms_rad;
  assert(peak > .01);
  assert(object.optical_diagnostic->failed_fits == 1);
  assert(object.optical_diagnostic->fits == 2);
  parameters[2].Pos[0] -= .2;
  survive_optimizer_current_norm(&optimizer);
  record_optical_diagnostic(&optimizer, 1, &user, true);
  assert(object.optical_diagnostic->rms_rad < 1e-6);
  assert(object.optical_diagnostic->peak_rms_rad == peak);

  mp_result success = {.status = 1, .bestnorm = .25};
  SurvivePose accepted = {0};
  assert(handle_optimizer_results(&optimizer, 1, &success, &user, NULL, &accepted) == .25);
  assert(memcmp(&accepted, &parameters[0], sizeof(accepted)) == 0);
  assert(object.optical_diagnostic->solver_status == 1);
  mp_result result = {.status = -1};
  SurvivePose output = {.Pos = {7, 8, 9}};
  const SurvivePose old_output = output;
  assert(handle_optimizer_results(&optimizer, -1, &result, &user, NULL, &output) < 0);
  assert(memcmp(&old_output, &output, sizeof(output)) == 0);
  assert(object.optical_diagnostic->failed_fits == 2);
  optimizer.stats.sensor_error = NAN;
  record_optical_diagnostic(&optimizer, -1, &user, true);
  assert(isnan(object.optical_diagnostic->rms_rad));
  assert(object.optical_diagnostic->invalid_fits == 1);
  optimizer.stats.sensor_error = 0;
  optimizer.stats.sensor_error_cnt = 0;
  record_optical_diagnostic(&optimizer, -1, &user, true);
  assert(isnan(object.optical_diagnostic->rms_rad));
  assert(object.optical_diagnostic->sequence == 7);
  object.optical_diagnostic = NULL;
  record_optical_diagnostic(&optimizer, 1, &user, true);
  assert(diagnostic.sequence == 7);
  object.optical_diagnostic = &diagnostic;
  survive_default_disconnect_process(&object);
  assert(object.optical_diagnostic->sequence == 0 && object.optical_diagnostic->fits == 0);
  survive_close(ctx);
}

static void test_measurement_time_reference(void)
{
  SurviveOpticalDiagnostic diagnostic = {.connected = true};
  SurviveObject object = {.optical_diagnostic = &diagnostic, .timebase_hz = 48000000};
  MPFITData data = {.opt = {.so = &object}};
  struct async_optimizer_user user = {.d = &data,
    .pdl = {.hdr = {.timecode = 47856000}}};
  survive_optimizer_measurement measurements[2] = {
    {.meas_type = survive_optimizer_measurement_type_light, .time = .990},
    {.meas_type = survive_optimizer_measurement_type_light, .time = .997}
  };
  survive_optimizer optimizer = {.measurements = measurements, .measurementsCnt = 2,
    .timecode = .995, .stats = {.sensor_error = .000002, .sensor_error_cnt = 2}};
  record_optical_diagnostic(&optimizer, 1, &user, true);
  assert(fabs(diagnostic.max_age_seconds - .007) < 1e-9);
  assert(fabs(diagnostic.rms_rad - .001) < 1e-9);
  assert(diagnostic.timecode == 47856000);
  assert(optimizer.timecode == .995 && measurements[1].time == .997);

  const FLT invalid_times[] = {NAN, INFINITY, -INFINITY, 1.1};
  for (size_t i = 0; i < sizeof(invalid_times) / sizeof(invalid_times[0]); ++i) {
    measurements[0].time = invalid_times[i];
    record_optical_diagnostic(&optimizer, 1, &user, true);
    assert(isnan(diagnostic.max_age_seconds));
    assert(isfinite(diagnostic.rms_rad));
  }
  measurements[0].time = .990;
  object.timebase_hz = 0;
  record_optical_diagnostic(&optimizer, 1, &user, true);
  assert(isnan(diagnostic.max_age_seconds));
  assert(isnan(diagnostic.window_seconds));
}

static void test_snapshot_lifetime(void)
{
  char *args[] = {"optical-lifetime-test", "--configfile", "/dev/null", "--no-save-config"};
  SurviveContext *ctx = survive_init_internal(4, args, NULL, NULL);
  assert(ctx && ctx->driver_ct == 0);
  survive_release_ctx_lock(ctx);
  ctx->buttonQueue.buttonservicesem = OGCreateSema();
  SurviveSimpleContext simple = {.ctx = ctx, .poll_mutex = OGCreateMutex(),
    .update_cv = OGCreateConditionVariable()};
  ctx->user_ptr = &simple;
  SurviveObject *native = calloc(1, sizeof(*native));
  native->ctx = ctx;
  survive_get_ctx_lock(ctx);
  new_object_fn(native);
  survive_release_ctx_lock(ctx);
  SurviveSimpleObject *object = simple.objects.head;
  assert(native->optical_diagnostic == &object->optical_diagnostic);
  *native->optical_diagnostic = (SurviveOpticalDiagnostic) {
    .connected = true, .sequence = 3, .fits = 10, .failed_fits = 1, .dropped_measurements = 3,
    .peak_rms_rad = .02, .peak_residual_count = 8, .residual_count = 8
  };
  SurviveOpticalDiagnostic sample;
  assert(survive_simple_get_optical_diagnostic(object, &sample));
  assert(sample.fits == 10 && sample.failed_fits == 1 && sample.peak_rms_rad == .02);
  assert(survive_simple_get_optical_diagnostic(object, &sample));
  assert(sample.sequence == 3 && sample.fits == 0 && sample.peak_residual_count == 0);
  survive_get_ctx_lock(ctx);
  survive_default_disconnect_process(native);
  survive_release_ctx_lock(ctx);
  free(native);
  assert(!survive_simple_get_optical_diagnostic(object, &sample));
  assert(sample.sequence == 0 && sample.fits == 0);
  SurviveObject *reconnected = calloc(1, sizeof(*reconnected));
  reconnected->ctx = ctx;
  survive_get_ctx_lock(ctx);
  new_object_fn(reconnected);
  survive_release_ctx_lock(ctx);
  assert(!survive_simple_get_optical_diagnostic(object, &sample));
  assert(survive_simple_get_optical_diagnostic(object->next, &sample));
  assert(sample.sequence == 0);
  survive_get_ctx_lock(ctx);
  survive_default_disconnect_process(reconnected);
  survive_release_ctx_lock(ctx);
  free(reconnected);
  free(object->next);
  free(object);
  OGDeleteMutex(simple.poll_mutex);
  OGDeleteConditionVariable(simple.update_cv);
  survive_get_ctx_lock(ctx);
  survive_close(ctx);
}

int main(void)
{
  test_measurement_time_reference();
  test_snapshot_lifetime();
  test_geometry(&survive_reproject_gen1_model, false);
  test_geometry(&survive_reproject_gen2_model, false);
  test_geometry(&survive_reproject_gen2_model, true);
  return 0;
}
