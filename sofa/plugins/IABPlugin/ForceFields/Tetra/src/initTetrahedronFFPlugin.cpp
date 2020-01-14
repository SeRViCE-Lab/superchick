#include "IABPlugin/ForceFields/Tetra/include/initTetrahedronFFPlugin.h"

// Ref sofa/v19.06/applications/plugins/SoftRobots/src/SoftRobots/component/initSoftRobots.cpp

extern "C" {

void initExternalModule()
{

}

const char* getModuleName()
{
  return "TetraToSphericalPolar";
}

const char* getModuleVersion()
{
  return "1.0";
}

const char* getModuleLicense()
{
  return "MIT";
}

const char* getModuleDescription()
{
  return "TetraSphericalPolar Plugin";
}

const char* getModuleComponentList()
{
  return "None for now";
}

}
