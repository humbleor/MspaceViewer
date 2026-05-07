#pragma once

#ifdef OSDBD_BIN_LIB
#define OSGB_BIN_EXPORT __declspec(dllexport)
#else
#define OSGB_BIN_EXPORT __declspec(dllimport)
#endif
