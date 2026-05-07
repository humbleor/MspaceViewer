#pragma once

#include <QtCore/qglobal.h>

#ifndef BUILD_STATIC
# if defined(OSGQT_LIB)
#  define OSGQT_EXPORT Q_DECL_EXPORT
# else
#  define OSGQT_EXPORT Q_DECL_IMPORT
# endif
#else
# define OSGQT_EXPORT
#endif
