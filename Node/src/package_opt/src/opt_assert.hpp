#pragma once

#include <stdio.h>

#ifndef NDEBUG
#define opt_assert(statement) {if (not (statement)) { \
fprintf(stderr, "Statement %s failed on line %i in file %s", #statement, __LINE__, __FILE__);}}
#else
#define opt_assert(statement) ; //allows to customize assertion behavior on target
#endif