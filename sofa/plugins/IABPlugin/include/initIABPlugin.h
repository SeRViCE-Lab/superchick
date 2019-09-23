
#ifndef INITIABPLUGIN_H
#define INITIABPLUGIN_H

#include <sofa/helper/system/config.h>
#define SOFA_TARGET IABPlugin
#ifdef SOFA_BUILD_IABPLUGIN
#define SOFA_IABPlugin_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#define SOFA_IABPlugin_API SOFA_IMPORT_DYNAMIC_LIBRARY
#endif


#endif
