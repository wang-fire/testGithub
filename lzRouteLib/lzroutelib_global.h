#ifndef LZROUTELIB_GLOBAL_H
#define LZROUTELIB_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(LZROUTELIB_LIBRARY)
#  define LZROUTELIBSHARED_EXPORT Q_DECL_EXPORT
#else
#  define LZROUTELIBSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // LZROUTELIB_GLOBAL_H
