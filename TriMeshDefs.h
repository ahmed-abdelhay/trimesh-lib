#pragma once

#include <qglobal.h>

#if defined TRIMESHLIB_LIBRARY
#  define TRIMESHLIBSHARED_DEFS Q_DECL_EXPORT
#else
#  define TRIMESHLIBSHARED_DEFS Q_DECL_IMPORT
#endif
