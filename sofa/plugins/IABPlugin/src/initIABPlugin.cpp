#include "IABPlugin/include/initIABPlugin.h"

// Ref sofa/v19.06/applications/plugins/SoftRobots/src/SoftRobots/component/initSoftRobots.cpp

extern "C" {

void initExternalModule()
{

}

const char* getModuleName()
{
  return "IAB";
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
  return "IAB Isochoric Plugin";
}

const char* getModuleComponentList()
{
  // see /home/lex/sofa/v19.06/applications/plugins/SoftRobots/src/SoftRobots/component/initSoftRobots.cpp
  return "None for now";
}

}
