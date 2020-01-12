// #define SOFA_COMPONENT_FORCEFIELD_TETRAHEDRONMOONEYRIVLINFEMFORCEFIELD_CPP
// #include "IABPlugißn/Forcefields/Tetra/include/initTetrahedronFFPlugin.h"
//
// // Ref sofa/v19.06/applications/plugins/SoftRobots/src/SoftRobots/component/initSoftRobots.cpp
//
// extern "C" {
//
// void initExternalModule()
// {
//
// }
//
// const char* getModuleName()
// {
//   return "IAB TetrahedronForceField";
// }
//
// const char* getModuleVersion()
// {
//   return "1.0";
// }
//
// const char* getModuleLicense()
// {
//   return "MIT";
// }
//
// const char* getModuleDescription()
// {
//   return "Tetrahedron Mooney-Rivlin Forcefield Plugin";
// }
//
// const char* getModuleComponentList()
// {
//   // see /home/lex/sofa/v19.06/applications/plugins/SoftRobots/src/SoftRobots/component/initSoftRobots.cpp
//   return "None for now";
// }
//
// }

#ifndef INITTETRAHEDRONMOONEYRIVLINFFPLUGIN_H
#define INITTETRAHEDRONMOONEYRIVLINFFPLUGIN_H

#include <sofa/helper/system/config.h>

#define SOFA_TARGET TetraMooneyRivlinPlugin
#ifdef SOFA_BUILD_TETRA_MOONRIV_PLUGIN
# define SOFA_TETRA_MOONRIV_Plugin_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
# define SOFA_TETRA_MOONRIV_Plugin_API SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

#endif
