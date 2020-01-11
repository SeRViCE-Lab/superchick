#ifndef INITTETRAHEDRONFFPLUGIN_H
#define INITTETRAHEDRONFFPLUGIN_H

#include <sofa/helper/system/config.h>

#define SOFA_TARGET IABTetrahedronPlugin
#ifdef SOFA_BUILD_IABTETRAHEDRONPLUGIN
# define SOFA_IABTetrahedronPlugin_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
# define SOFA_IABTetrahedronPlugin_API SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

#endif
