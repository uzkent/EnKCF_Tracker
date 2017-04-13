/// Copyright (c) 2016 AutelRobotics. All Rights Reserved
///
/// \wrappers.h
///
/// \Definition : aligned memory allocation
///
///  \author XipengCui
///  \Date : Created on: Jan 1, 2017
///
#ifndef _WRAPPERS_HPP_
#define _WRAPPERS_HPP_

#include <stdlib.h>

// platform independent aligned memory allocation (see also alFree)
static void* alMalloc(size_t size, int alignment) {
  const size_t pSize = sizeof(void*), a = alignment-1;
  void *raw = malloc(size + a + pSize);
  void *aligned = (void*) (((size_t) raw + pSize + a) & ~a);
  *(void**) ((size_t) aligned-pSize) = raw;
  return aligned;
}

// platform independent alignned memory de-allocation (see also alMalloc)
static void alFree(void* aligned) {
  void* raw = *(void**)((char*)aligned-sizeof(void*));
  free(raw);
}

#endif
