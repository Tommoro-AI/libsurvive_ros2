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
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "redist/os_generic.h"

static pthread_mutex_t gate_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t gate_changed = PTHREAD_COND_INITIALIZER;
static bool worker_waiting;

static void observed_wait(og_cv_t condition, og_mutex_t mutex)
{
  pthread_mutex_lock(&gate_lock);
  worker_waiting = true;
  pthread_cond_broadcast(&gate_changed);
  pthread_mutex_unlock(&gate_lock);
  OGWaitCond(condition, mutex);
}

// Observe the worker's wait boundary to schedule the race without sleeps or device drivers.
#define OGWaitCond observed_wait
#include "driver_global_scene_solver.c"  // NOLINT(build/include): exercise upstream static handlers.
#undef OGWaitCond
static bool solve_entered;
static bool release_solve;
static bool freeze_returned;
static unsigned solve_calls;
static unsigned callbacks[4];

static void on_imu(SurviveObject *so, int mask, const FLT *values, survive_timecode time, int id)
{
  (void)so; (void)mask; (void)values; (void)time; (void)id;
  ++callbacks[0];
}
static void on_sync(
  SurviveObject *so, survive_channel channel, survive_timecode time, bool ootx,
  bool gen)
{
  (void)so; (void)channel; (void)time; (void)ootx; (void)gen;
  ++callbacks[1];
}
static void on_light(
  SurviveObject *so, int sensor, int code, survive_timecode time, FLT length,
  uint32_t lh)
{
  (void)so; (void)sensor; (void)code; (void)time; (void)length; (void)lh;
  ++callbacks[2];
}
static void on_ootx(SurviveContext *ctx, uint8_t index)
{
  (void)ctx; (void)index;
  ++callbacks[3];
}

static int blocked_poser(SurviveObject *so, PoserData *data)
{
  assert(data->pt == POSERDATA_GLOBAL_SCENES);
  ++solve_calls;
  survive_release_ctx_lock(so->ctx);
  pthread_mutex_lock(&gate_lock);
  solve_entered = true;
  pthread_cond_broadcast(&gate_changed);
  while (!release_solve) {
    pthread_cond_wait(&gate_changed, &gate_lock);
  }
  pthread_mutex_unlock(&gate_lock);
  survive_get_ctx_lock(so->ctx);
  return 0;
}

static void * freeze_thread(void *context)
{
  assert(survive_freeze_calibration(context));
  pthread_mutex_lock(&gate_lock);
  freeze_returned = true;
  pthread_cond_broadcast(&gate_changed);
  pthread_mutex_unlock(&gate_lock);
  return NULL;
}

static SurviveContext * make_context(void)
{
  char *args[] = {"calibration-freeze-test", "--configfile", "/dev/null", "--no-save-config"};
  SurviveContext *ctx = survive_init_internal(4, args, NULL, NULL);
  assert(ctx != NULL);
  assert(ctx->driver_ct == 0);
  ctx->buttonQueue.buttonservicesem = OGCreateSema();
  survive_release_ctx_lock(ctx);
  return ctx;
}

static void mark_calibrated(SurviveContext *ctx)
{
  ctx->activeLighthouses = 1;
  ctx->bsd[0].BaseStationID = 0x1234;
  ctx->bsd[0].PositionSet = true;
  ctx->bsd[0].OOTXSet = true;
  ctx->bsd[0].Pose.Rot[0] = 1;
  ctx->bsd[0].Pose.Pos[0] = 1.25;
}

int main(void)
{
  assert(!survive_freeze_calibration(NULL));
  SurviveContext *ctx = make_context();
  assert(!survive_freeze_calibration(ctx));
  assert(!ctx->calibration_frozen);
  mark_calibrated(ctx);
  ctx->bsd[0].OOTXSet = false;
  assert(!survive_freeze_calibration(ctx));
  ctx->bsd[0].OOTXSet = true;
  ctx->bsd[0].disable = true;
  assert(!survive_freeze_calibration(ctx));
  ctx->bsd[0].disable = false;
  assert(survive_freeze_calibration(ctx));
  assert(survive_freeze_calibration(ctx));
  survive_close(ctx);

  ctx = make_context();
  mark_calibrated(ctx);
  survive_install_imu_fn(ctx, on_imu);
  survive_install_sync_fn(ctx, on_sync);
  survive_install_light_pulse_fn(ctx, on_light);
  survive_install_ootx_received_fn(ctx, on_ootx);
  ctx->PoserFn = blocked_poser;
  SurviveObject object = {.ctx = ctx};
  object.OutPoseIMU.Rot[0] = 1;
  object.OutPoseIMU.Pos[0] = 3.5;
  ctx->objs = calloc(1, sizeof(*ctx->objs));
  ctx->objs[0] = &object;
  ctx->objs_ct = 1;
  assert(DriverRegGlobalSceneSolver(ctx) == SURVIVE_DRIVER_PASSIVE);
  global_scene_solver *gss = ctx->drivers[0];
  assert(gss->threaded);
  BaseStationData station_before;
  memcpy(&station_before, &ctx->bsd[0], sizeof(station_before));
  const SurvivePose tracker_before = object.OutPoseIMU;
  const void *filter_before = ctx->bsd[0].tracker;

  pthread_mutex_lock(&gate_lock);
  while (!worker_waiting) {
    pthread_cond_wait(&gate_changed, &gate_lock);
  }
  pthread_mutex_unlock(&gate_lock);
  OGLockMutex(gss->data_available_lock);
  gss->needsSolve = true;
  OGSignalCond(gss->data_available);
  OGUnlockMutex(gss->data_available_lock);
  pthread_mutex_lock(&gate_lock);
  while (!solve_entered) {
    pthread_cond_wait(&gate_changed, &gate_lock);
  }
  pthread_mutex_unlock(&gate_lock);

  survive_get_ctx_lock(ctx);
  ctx->ootx_receivedproc(ctx, 0);
  survive_release_ctx_lock(ctx);
  pthread_t freezing;
  assert(pthread_create(&freezing, NULL, freeze_thread, ctx) == 0);
  for (;; ) {
    survive_get_ctx_lock(ctx);
    bool frozen = ctx->calibration_frozen;
    survive_release_ctx_lock(ctx);
    if (frozen) {
      break;
    }
    sched_yield();
  }
  pthread_mutex_lock(&gate_lock);
  assert(!freeze_returned && "Freeze returned before the running optimizer finished");
  release_solve = true;
  pthread_cond_broadcast(&gate_changed);
  pthread_mutex_unlock(&gate_lock);
  assert(pthread_join(freezing, NULL) == 0);
  assert(solve_calls == 1 && "A queued solve ran after calibration was frozen");
  assert(memcmp(&station_before, &ctx->bsd[0], sizeof(station_before)) == 0);
  assert(memcmp(&tracker_before, &object.OutPoseIMU, sizeof(tracker_before)) == 0);
  assert(ctx->bsd[0].tracker == filter_before);

  survive_get_ctx_lock(ctx);
  ctx->imuproc(&object, 0, NULL, 0, 0);
  ctx->syncproc(&object, 0, 0, false, false);
  ctx->light_pulseproc(&object, 0, 0, 0, 0, 0);
  ctx->ootx_receivedproc(ctx, 0);
  survive_release_ctx_lock(ctx);
  assert(callbacks[0] == 1 && callbacks[1] == 1 && callbacks[2] == 1 && callbacks[3] == 2);
  assert(solve_calls == 1);
  assert(survive_freeze_calibration(ctx));
  ctx->objs_ct = 0;
  survive_close(ctx);
  pthread_cond_destroy(&gate_changed);
  pthread_mutex_destroy(&gate_lock);
  return 0;
}
