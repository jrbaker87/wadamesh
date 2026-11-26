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
#include "esp_memory_utils.h"

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
  Acc refr;                                // LVGL's own refresh (render+flush), ms*1000
  uint32_t passRefrMs = 0;                 // refresh time reported inside the current pass
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

// Per-frame breakdown of the double-buffered flush path (RM69A10Display.cpp), microseconds.
// Filled while LVGL flushes the bands of one refresh, printed and cleared by onRefr().
struct Frame {
  int64_t wait = 0;       // dbOpen: waiting for the previous swap to land
  int64_t rowcopy = 0;    // copying rows the front has and the back missed (per band + at present)
  int64_t upscale = 0;    // the 2x nearest-neighbour write of the band into the back buffer
  int64_t writeback = 0;  // esp_lcd_panel_draw_bitmap: cache write-back + buffer select
  uint32_t rows = 0;      // rows copied front -> back
  uint32_t bands = 0;
};
inline Frame fr;

// Per-frame time inside LVGL's draw hooks, by kind (UITask.cpp installs the wrappers).
enum DrawKind { DK_RECT_SHADOW, DK_RECT_GRAD, DK_RECT_ROUND, DK_RECT_PLAIN, DK_LETTER, DK_IMG,
                DK_LINE, DK_ARC, DK_POLY, DK_LAYER, DK_COUNT };
inline const char* const kDrawKindName[DK_COUNT] = {"shadow", "grad", "round", "rect", "text", "img",
                                                     "line", "arc", "poly", "layer"};
struct DrawProf { int64_t us[DK_COUNT] = {}; uint32_t n[DK_COUNT] = {}; };
inline DrawProf dp;
inline void drawAdd(DrawKind k, int64_t us) { dp.us[k] += us; ++dp.n[k]; }

// One band went to the panel. up_us = CPU upscale, dma_us = draw_bitmap + wait for the copy.
inline void onFlush(int64_t up_us, int64_t dma_us, bool timedOut, uint32_t px) {
  g.up.add((uint64_t)up_us);
  g.dma.add((uint64_t)dma_us);
  g.flushPxWin += px;
  ++g.flushesTotal;
  ++g.flushesWin;
  if (timedOut) ++g.dmaTimeoutsWin;
}

// LVGL finished one display refresh (disp_drv.monitor_cb): ms = time of the whole refresh
// (render + flush of every band). Pass time minus this is spent in OTHER lv_timers
// (animations, app timers) and anything those call.
inline void onRefr(uint32_t ms, uint32_t /*px*/) {
  g.refr.add((uint64_t)ms * 1000u);
  g.passRefrMs += ms;
  if (ms >= 40) {
    const double flush = (double)(fr.wait + fr.rowcopy + fr.upscale + fr.writeback) / 1000.0;
    printf("[P4PERF]   frame %u ms (%u bands): lvgl draw %.1f | wait-swap %.1f | row-copy %.1f (%u rows) | "
           "upscale %.1f | writeback %.1f\n",
           (unsigned)ms, (unsigned)fr.bands, (double)ms - flush, fr.wait / 1000.0, fr.rowcopy / 1000.0,
           (unsigned)fr.rows, fr.upscale / 1000.0, fr.writeback / 1000.0);
    char line[256]; int n = 0;
    for (int k = 0; k < DK_COUNT && n < (int)sizeof line - 32; ++k)
      if (dp.n[k]) n += snprintf(line + n, sizeof line - n, " %s %.1f/%u", kDrawKindName[k],
                                 dp.us[k] / 1000.0, (unsigned)dp.n[k]);
    printf("[P4PERF]     draw ms/calls:%s\n", n ? line : " (no hooks)");
  }
  fr = Frame{};
  dp = DrawProf{};
}

// One lv_timer_handler() call finished. flushesBefore = g.flushesTotal sampled before the call.
inline void onLvglPass(int64_t us, uint32_t flushesBefore) {
  const uint32_t bands = g.flushesTotal - flushesBefore;
  const uint32_t refrMs = g.passRefrMs;
  g.passRefrMs = 0;
  if (bands == 0) return;                  // idle call: nothing was drawn, not a frame
  g.pass.add((uint64_t)us);
  ++g.passesWin;
  g.bandsInPassesWin += bands;
  if (us > 40000 && (now() - g.lastSlowUs) > 100000) {
    g.lastSlowUs = now();
    const double other = (double)us / 1000.0 - (double)refrMs;
    printf("[P4PERF] slow pass %.1f ms (%u bands) = lvgl refresh %u ms + other timers/callbacks %.0f ms\n",
           (double)us / 1000.0, (unsigned)bands, (unsigned)refrMs, other);
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
    printf("[P4PERF] pass/s=%.1f bands/pass=%.1f | pass %.1f/%.1f ms (refr avg %.1f max %.0f) | upscale %.2f/%.2f ms | "
           "dma-wait %.2f/%.2f ms to=%u | loop ui %.1f/%.1f ms rest %.1f/%.1f ms | "
           "heap int=%uK big=%uK psram=%uK\n",
           (double)g.passesWin / secs, bandsPerPass,
           g.pass.avgMs(), g.pass.maxMs(), g.refr.avgMs(), g.refr.maxMs(),
           g.up.avgMs(), g.up.maxMs(),
           g.dma.avgMs(), g.dma.maxMs(), (unsigned)g.dmaTimeoutsWin,
           g.loopUi.avgMs(), g.loopUi.maxMs(),
           g.loopRest.avgMs(), g.loopRest.maxMs(),
           (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
           (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024),
           (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
  }
  g.up.reset(); g.dma.reset(); g.pass.reset(); g.refr.reset(); g.loopUi.reset(); g.loopRest.reset();
  g.flushesWin = 0; g.flushPxWin = 0; g.dmaTimeoutsWin = 0; g.passesWin = 0; g.bandsInPassesWin = 0;
}

// RAII timer for code the UI runs on one page change. Prints "[P4PERF]   build <name> N ms" when the
// scope took longer than thr_us (default 4 ms), so a log names the slow steps of a transition.
struct Scope {
  const char* n; int64_t t0; int64_t thr;
  explicit Scope(const char* name, int64_t thr_us = 4000) : n(name), t0(now()), thr(thr_us) {}
  ~Scope() {
    const int64_t d = now() - t0;
    if (d > thr) printf("[P4PERF]   build %s %.1f ms\n", n, (double)d / 1000.0);
  }
};

}  // namespace p4perf

#define P4PERF_CAT2(a, b) a##b
#define P4PERF_CAT(a, b) P4PERF_CAT2(a, b)
#define P4PERF_SCOPE(name) p4perf::Scope P4PERF_CAT(p4perf_scope_, __LINE__)(name)

#else
#define P4PERF_SCOPE(name) ((void)0)
#endif  // TDP4_PERF_TRACE
