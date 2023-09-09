#pragma once

#define min(a, b) \
  ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#define max(a, b) \
  ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define constrain(val, min, max) \
  ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

#define map(val, in_min, in_max, out_min, out_max) \
  ((constrain(val, in_min, in_max) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))
