#include <include/initIABPlugin.h>


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
  return "Dummy Description";
}

const char* getModuleComponentList()
{
  return "None for now";
}

}
