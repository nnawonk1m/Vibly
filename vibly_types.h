/* ============================================================================
 * vibly_types.h
 * ----------------------------------------------------------------------------
 * Type definitions for the haptic engine.
 *
 * WHY THESE LIVE IN A HEADER RATHER THAN IN THE .ino
 *
 * The Arduino IDE preprocesses a sketch before handing it to the compiler:
 * it scans for function definitions and injects a prototype for each one near
 * the top of the file. That injection point sits above almost all of the
 * sketch body, so any function whose signature mentions a type declared in
 * the .ino gets a prototype that references a type the compiler has not seen
 * yet:
 *
 *     static void hapticSetSourceLocked(SoundClass, float, float);   <-- injected
 *     ...
 *     enum SoundClass : uint8_t { ... };                             <-- too late
 *
 *   -> error: 'SoundClass' was not declared in this scope
 *
 * #include lines are left where they are, above the injection point, so a
 * type that arrives through a header is always visible to the prototypes.
 * This is the standard fix and it is the reason enums and structs used in
 * function signatures belong in a header in any Arduino project.
 * ==========================================================================*/
#ifndef VIBLY_TYPES_H
#define VIBLY_TYPES_H

#include <stdint.h>

/* Haptic classes. These are the DEVICE's vocabulary, deliberately kept
 * separate from the model's label indices — the mapping between them lives
 * in CLASS_MAP in the sketch, so retraining the model with different or
 * reordered labels only touches that one table. */
enum SoundClass : uint8_t {
  CLASS_NONE = 0,
  CLASS_SPEECH,
  CLASS_CAR_HORN,
  CLASS_BABY_CRY,
  CLASS_COUNT
};

/* One ON or OFF span of a vibration pattern. */
struct Segment { bool on; uint16_t ms; };

struct Pattern {
  const Segment* seg;
  uint8_t        count;
  const char*    name;
};

/* ONE shared clock drives BOTH motors. Only the duty differs per side, so
 * the rhythm can never drift apart between left and right. */
struct HapticState {
  SoundClass cls;
  uint8_t    segIdx;
  uint32_t   segStart;
  bool       kicking;
  uint32_t   kickStart;
  uint8_t    dutyL;       /* target sustain duty, left  */
  uint8_t    dutyR;       /* target sustain duty, right */
  uint8_t    liveL;       /* what is actually on the pin right now */
  uint8_t    liveR;
};

#endif /* VIBLY_TYPES_H */
