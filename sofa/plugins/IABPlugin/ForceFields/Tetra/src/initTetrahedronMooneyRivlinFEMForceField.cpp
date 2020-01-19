#include "IABPlugin/ForceFields/Tetra/include/initTetrahedronMooneyRivlinFEMForceField.h"

// Ref sofa/v19.06/applications/plugins/SoftRobots/src/SoftRobots/component/initSoftRobots.cpp

extern "C" {

void initExternalModule()
{

}

const char* getModuleName()
{
  return "TetrahedronMooneyRivlinFEMForceField";
}

const char* getModuleVersion()
{
  return "1.0";
}

const char* getModuleLicense()
{
  return "BSD";
}

const char* getModuleDescription()
{
  return "TetrahedronMooneyRivlinFEMForceField to Spherical Coords Plugin";
}

const char* getModuleComponentList()
{
  return "None for now";
}

}
