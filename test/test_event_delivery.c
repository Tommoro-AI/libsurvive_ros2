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
#include <stdbool.h>

#include "redist/os_generic.h"

static unsigned wait_calls;
static bool observed_wait(og_cv_t condition, og_mutex_t mutex, int timeout_ms)
{
  (void)condition; (void)mutex;
  assert(timeout_ms == 100);
  ++wait_calls;
  return false;
}

// Observe waiting deterministically without depending on scheduler latency.
#define OGWaitCondTimeout observed_wait
#include "survive_api.c"  // NOLINT(build/include): exercise upstream static handlers.
#undef OGWaitCondTimeout

static void check_pending(SurviveSimpleContext *context, SurviveSimpleObject *object)
{
  SurviveSimpleEvent event = {.event_type = SurviveSimpleEventType_ButtonEvent};
  OGLockMutex(context->poll_mutex);
  insert_into_event_buffer(context, &event);
  assert(survive_simple_wait_for_event(context, &event) == SurviveSimpleEventType_ButtonEvent);
  assert(wait_calls == 0);

  object->has_update = true;
  assert(survive_simple_wait_for_event(context, &event) == SurviveSimpleEventType_PoseUpdateEvent);
  assert(event.d.pose_event.object == object);
  assert(wait_calls == 0);
  assert(survive_simple_next_event(context, &event) == SurviveSimpleEventType_None);
  assert(survive_simple_wait_for_event(context, &event) == SurviveSimpleEventType_None);
  assert(wait_calls == 1);

  context->running = false;
  assert(survive_simple_wait_for_event(context, &event) == SurviveSimpleEventType_Shutdown);
  assert(wait_calls == 1);
  context->running = true;
}

static void check_fairness(
  SurviveSimpleContext *context, SurviveSimpleObject *first,
  SurviveSimpleObject *second)
{
  first->has_update = second->has_update = true;
  assert(survive_simple_get_next_updated(context) == first);
  for (unsigned iteration = 0; iteration < 100; ++iteration) {
    first->has_update = true;
    assert(survive_simple_get_next_updated(context) == second);
    second->has_update = true;
    assert(survive_simple_get_next_updated(context) == first);
  }
  assert(survive_simple_get_next_updated(context) == second);
  assert(survive_simple_get_next_updated(context) == NULL);
  first->has_update = true;
  assert(survive_simple_get_next_updated(context) == first);
  assert(survive_simple_get_next_updated(context) == NULL);
}

int main(void)
{
  SurviveSimpleContext context = {.running = true, .poll_mutex = OGCreateMutex(),
    .update_cv = OGCreateConditionVariable()};
  SurviveObject tracker = {.OutPose = {.Rot = {1}}, .OutPose_timecode = 48000000};
  SurviveSimpleObject first = {.actx = &context, .type = SurviveSimpleObject_OBJECT,
    .data.so = &tracker};
  SurviveSimpleObject second = {.actx = &context, .type = SurviveSimpleObject_OBJECT,
    .data.so = &tracker};
  SurviveSimpleObjectList_add(&context.objects, &first);
  SurviveSimpleObjectList_add(&context.objects, &second);
  check_fairness(&context, &first, &second);
  check_pending(&context, &first);
  OGDeleteConditionVariable(context.update_cv);
  OGDeleteMutex(context.poll_mutex);
  return 0;
}
