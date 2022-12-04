#pragma once
#include "stdio.h"

#define opt_assert(statement) { \
if (not (statement)) fprintf(stderr, "Assertion failed in in %s, line %i", __FILE__, __LINE__); \
}
