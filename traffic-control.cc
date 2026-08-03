#include "traffic-control.h"

// Slice enforcement is at the control plane (allocate shares <= capacity, rate-shape
// each source to its share), not the MAC. The slice->ToS map lives in the header;
// no code needed here.
