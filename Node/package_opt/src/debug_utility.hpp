#pragma once
#include "stdio.h"

#ifndef NDEBUG
#define opt_assert(statement) { \
if (not (statement)) fprintf(stderr, "Assertion failed in in %s, line %i", __FILE__, __LINE__); \
}
#else
#define opt_assert(statement) ;
#endif //NDEBUG opt_assert
