// SPDX-License-Identifier: GPL-3.0-or-later
//
// Opt-in frame/flush profiler for the LilyGo T-Display P4.
//
//   WADA_P4_PERF_TRACE=1 ./build.sh build        (add WADA_P4_LCD=1 for the TFT-LCD SKU)
//
// Compiles to NOTHING unless TDP4_PERF_TRACE is defined, so normal and release builds are
// byte-for-byte unchanged. It exists to turn "the page change feels slow / tears" into numbers
// BEFORE any optimisation is attempted, and to re-measure after each one.
//
// What one console line per second tells you (grep the serial log for [P4PERF]):
//   pass/s      LVGL render passes per second that actually drew something (≈ achieved fps)
//   bands/pass  flush_cb calls per pass. A full-screen repaint is ~26 bands of 24 rows at the
//               half-res 284x616 UI, so animation frames that repaint everything sit near 26.
//   pass        time inside lv_timer_handler for those passes (render + flush, avg / worst)
//   upscale     CPU nearest-neighbour 2x upscale of each band (avg / worst, per band)
//   dma-wait    time blocked in draw_bitmap + waiting for the DMA2D copy into the framebuffer
//               (avg / worst, per band). A "to=" above 0 means the 100 ms semaphore timed out,
//               i.e. a 100 ms freeze of the UI task.
//   loop ui     whole ui_task.loop() including lv_timer_handler and everything the UI does
//   loop rest   Wi-Fi state machine + sensors + mesh in app_main's loop. A large WORST here is
//               something other than rendering stalling the UI task (blocking AT/socket calls,
//               the delay(50) calls around Wi-Fi mode changes, mesh/radio work).
//   heap        free internal RAM, largest internal block, free PSRAM
//
// Single slow passes are also logged on their own line ("slow pass"), rate-limited, so a page
// build or a transition spike is visible even if the one-second average hides it.
//
// All hooks run on the UI task (lvglFlush is called from lv_timer_handler on the same task), so
// there is no locking. The printf cost is one short line per second plus the rate-limited
// slow-pass lines; it will perturb timing slightly, which is why it is a build-time opt-in.
#pragma once

#if defined(TDP4_PERF_TRACE)

#include <stdint.h>
#include <stdio.h>
#include "esp_timer.h"
#include "esp_heap_caps.h"

namespace p4perf {

struct Acc {
  uint64_t sum = 0, max = 0;
  uint32_t n = 0;
  void   add(uint64_t v)  { sum += v; if (v > max) max = v; ++n; }
  double avgMs() const    { return n ? (double)sum / (double)n / 1000.0 : 0.0; }
  double maxMs() const    { return (double)max / 1000.0; }
  void   reset()          { sum = 0; max = 0; n = 0; }
};

struct State {
  Acc up, dma, pass, loopUi, loopRest;     // microseconds
  uint32_t flushesTotal = 0;               // monotonic, used to count bands inside one pass
  uint32_t flushesWin = 0;                 // this one-second window
  uint32_t flushPxWin = 0;
  uint32_t dmaTimeoutsWin = 0;
  uint32_t passesWin = 0;
  uint32_t bandsInPassesWin = 0;
  int64_t  lastPrintUs = 0;
  int64_t  lastSlowUs = 0;
};

inline State g;

inline int64_t now() { return esp_timer_get_time(); }

// One band went to the panel. up_us = CPU upscale, dma_us = draw_bitmap + wait for the copy.
inline void onFlush(int64_t up_us, int64_t dma_us, bool timedOut, uint32_t px) {
  g.up.add((uint64_t)up_us);
  g.dma.add((uint64_t)dma_us);
  g.flushPxWin += px;
  ++g.flushesTotal;
  ++g.flushesWin;
  if (timedOut) ++g.dmaTimeoutsWin;
}

// One lv_timer_handler() call finished. flushesBefore = g.flushesTotal sampled before the call.
inline void onLvglPass(int64_t us, uint32_t flushesBefore) {
  const uint32_t bands = g.flushesTotal - flushesBefore;
  if (bands == 0) return;                  // idle call: nothing was drawn, not a frame
  g.pass.add((uint64_t)us);
  ++g.passesWin;
  g.bandsInPassesWin += bands;
  if (us > 40000 && (now() - g.lastSlowUs) > 100000) {
    g.lastSlowUs = now();
    printf("[P4PERF] slow pass %.1f ms (%u bands)\n", (double)us / 1000.0, (unsigned)bands);
  }
}

// One iteration of app_main's loop: ui_task.loop() time and everything else in the iteration.
inline void onLoop(int64_t ui_us, int64_t rest_us) {
  g.loopUi.add((uint64_t)ui_us);
  g.loopRest.add((uint64_t)rest_us);
}

// Call once per loop iteration; prints and resets the window about once a second.
inline void tick() {
  const int64_t t = now();
  if (g.lastPrintUs == 0) { g.lastPrintUs = t; return; }
  const int64_t dt = t - g.lastPrintUs;
  if (dt < 1000000) return;
  g.lastPrintUs = t;

  const bool active = g.passesWin || g.loopRest.maxMs() > 20.0 || g.dmaTimeoutsWin;
  if (active) {
    const double secs = (double)dt / 1e6;
    const double bandsPerPass = g.passesWin ? (double)g.bandsInPassesWin / (double)g.passesWin : 0.0;
    printf("[P4PERF] pass/s=%.1f bands/pass=%.1f | pass %.1f/%.1f ms | upscale %.2f/%.2f ms | "
           "dma-wait %.2f/%.2f ms to=%u | loop ui %.1f/%.1f ms rest %.1f/%.1f ms | "
           "heap int=%uK big=%uK psram=%uK\n",
           (double)g.passesWin / secs, bandsPerPass,
           g.pass.avgMs(), g.pass.maxMs(),
           g.up.avgMs(), g.up.maxMs(),
           g.dma.avgMs(), g.dma.maxMs(), (unsigned)g.dmaTimeoutsWin,
           g.loopUi.avgMs(), g.loopUi.maxMs(),
           g.loopRest.avgMs(), g.loopRest.maxMs(),
           (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
           (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024),
           (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
  }
  g.up.reset(); g.dma.reset(); g.pass.reset(); g.loopUi.reset(); g.loopRest.reset();
  g.flushesWin = 0; g.flushPxWin = 0; g.dmaTimeoutsWin = 0; g.passesWin = 0; g.bandsInPassesWin = 0;
}

}  // namespace p4perf

#endif  // TDP4_PERF_TRACE
