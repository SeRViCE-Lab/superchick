#include <sstream>
using std::ostringstream ;
#include <fstream>

#include <string>
using std::string;

#include <vector>
using std::vector;

#include "IABPlugin/include/debuggers.h"
#include "IABPlugin/ForceFields/include/initIABPlugin.h"
#include <sofa/helper/ArgumentParser.h>
#include <SofaSimulationCommon/common.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/simulation/Node.h>
#include <sofa/helper/system/PluginManager.h>
#include <sofa/simulation/config.h> // #defines SOFA_HAVE_DAG (or not)
#include <SofaSimulationCommon/init.h>
#ifdef SOFA_HAVE_DAG
#include <SofaSimulationGraph/init.h>
#include <SofaSimulationGraph/DAGSimulation.h>
#endif

using sofa::simulation::Node;
#include <sofa/simulation/SceneLoaderFactory.h>
#include <SofaGraphComponent/SceneCheckerListener.h>
using sofa::simulation::scenechecking::SceneCheckerListener;

#include <SofaComponentCommon/initComponentCommon.h>
#include <SofaComponentBase/initComponentBase.h>
#include <SofaComponentGeneral/initComponentGeneral.h>
#include <SofaComponentAdvanced/initComponentAdvanced.h>
#include <SofaComponentMisc/initComponentMisc.h>

#include <SofaGeneralLoader/ReadState.h>
#include <SofaValidation/CompareState.h>
#include <sofa/helper/Factory.h>
#include <sofa/helper/cast.h>
#include <sofa/helper/BackTrace.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/FileSystem.h>
using sofa::helper::system::FileSystem;
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/helper/Utils.h>
#include <sofa/gui/GUIManager.h>
using sofa::gui::GUIManager;

#include <sofa/gui/Main.h>
#include <sofa/gui/BatchGUI.h>  // needed for BaseGUI
#include <sofa/helper/system/gl.h>
// #include <sofa/helper/system/atomic.h>

using sofa::core::ExecParams ;

#include <sofa/helper/system/console.h>
using sofa::helper::Utils;

using sofa::component::misc::CompareStateCreator;
using sofa::component::misc::ReadStateActivator;
using sofa::simulation::graph::DAGSimulation;
using sofa::helper::system::SetDirectory;
using sofa::core::objectmodel::BaseNode ;
using sofa::gui::BatchGUI;
using sofa::gui::BaseGUI;

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/logging/Messaging.h>

#include <sofa/gui/GuiDataRepository.h>
using sofa::gui::GuiDataRepository ;

using sofa::helper::system::DataRepository;
using sofa::helper::system::PluginRepository;
using sofa::helper::system::PluginManager;

using namespace sofa::defaulttype;

// see http://www.decompile.com/cpp/faq/file_and_line_error_string.htm
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define AT __FILE__ ":" TOSTRING(__LINE__)

// imported from cmakelists
#define SOFA SOFA_ROOT
// #define IAB_ROOT IAB_ROOT

void loadVerificationData(std::string& directory, std::string& filename, Node* node)
{
    msg_info("") << "loadVerificationData from " << directory << " and file " << filename ;

    string refFile;

    refFile += directory;
    refFile += '/';
    refFile += SetDirectory::GetFileName(filename.c_str());

    msg_info("") << "loadVerificationData " << refFile ;

    CompareStateCreator compareVisitor(ExecParams::defaultInstance());
    compareVisitor.setCreateInMapping(true);
    compareVisitor.setSceneName(refFile);
    compareVisitor.execute(node);

    ReadStateActivator v_read(ExecParams::defaultInstance(), true);
    v_read.execute(node);
}

void addGUIParameters(ArgumentParser* argumentParser)
{
    GUIManager::RegisterParameters(argumentParser);
}


int main(int argc, char** argv)
{
    auto SofaBuildPath = std::string(TOSTRING(SOFA));
    std::string IAB_PATH = std::string(TOSTRING(IAB_ROOT));
    std::string IABBuildPATH = IAB_PATH + "/build";

    std::string dir = SofaBuildPath + "/install/share/sofa/gui/qt/";
    dir = SetDirectory::GetRelativeFromProcess(dir.c_str());
    if(FileSystem::isDirectory(dir))
    {
        sofa::gui::GuiDataRepository.addFirstPath(dir);
    }

    // Add plugins and modules dirs to PluginRepository
    if ( FileSystem::isDirectory(Utils::getSofaPathPrefix()+"/plugins") )
    {
        PluginRepository.addFirstPath(Utils::getSofaPathPrefix()+"/plugins");
    }

    sofa::helper::BackTrace::autodump();
    ExecParams::defaultInstance()->setAspectID(0);
    sofa::gui::initMain();

    // string fileName ;
    bool noSceneCheck, temporaryFile = false;
    bool guiViz = false;  // run the gui scene
    unsigned int nbMSSASamples = 1;
    unsigned int computationTimeSampling=1; ///< Frequency of display of the computation time statistics, in number of animation steps. 0 means never.
    string    computationTimeOutputType="stdout";
    string gui,  verif = "";
    string simulationType = "dag";

    vector<string> plugins, files;
#ifdef SOFA_SMP
    string nProcs="";
    bool        disableStealing, affinity = false;
#endif
    string colorsStatus = "unset";
    string messageHandler = "auto";
    bool enableInteraction = false ;
    int width = 1280;
    int height = 1024;

    ArgumentParser* argParser = new ArgumentParser(argc, argv);
    argParser->addArgument(po::value<std::vector<std::string>>(&plugins), "load,l", "load given plugins");
    argParser->addArgument(po::value<std::string>(&simulationType),  "simu,s", "select the type of simulation (bgl, dag, tree)");
    argParser->addArgument(po::value<std::string>(&verif)->default_value(""), "verification,v","load verification data for the scene");
    argParser->addArgument(po::value<std::string>(&colorsStatus)->default_value("unset", "auto")->implicit_value("yes"),"colors,c", "use colors on stdout and stderr (yes, no, auto)");
    argParser->addArgument(po::value<std::string>(&messageHandler)->default_value("auto"), "formatting,f","select the message formatting to use (auto, clang, sofa, rich, test)");
    argParser->addArgument(po::value<bool>(&enableInteraction)->default_value(false)->implicit_value(true),"interactive,i", "enable interactive mode for the GUI which includes idle and mouse events (EXPERIMENTAL)");
    argParser->addArgument(po::value<bool>(&guiViz)->default_value(false)->implicit_value(true),"visualize,g", "display gui window at startup");
    argParser->addArgument(po::value<std::vector<std::string> >()->multitoken(), "argv","forward extra args to the python interpreter");

#ifdef SOFA_SMP
    argParser->addArgument(po::value<bool>(&disableStealing)->default_value(false)->implicit_value(true),           "disableStealing,w", "Disable Work Stealing")
    argParser->addArgument(po::value<std::string>(&nProcs)->default_value(""),                                      "nprocs", "Number of processor")
    argParser->addArgument(po::value<bool>(&affinity)->default_value(false)->implicit_value(true),                  "affinity", "Enable aFfinity base Work Stealing")
#endif

    addGUIParameters(argParser);
    argParser->parse();
    files = argParser->getInputFileList();
#ifdef SOFA_HAVE_DAG
    sofa::simulation::graph::init();
#endif
    sofa::component::initComponentBase();
    sofa::component::initComponentCommon();
    sofa::component::initComponentGeneral();
    sofa::component::initComponentAdvanced();
    sofa::component::initComponentMisc();

    sofa::simulation::setSimulation(new DAGSimulation());

    // Output FileRepositories
    msg_info("IAB") << "PluginRepository paths = " << PluginRepository.getPathsJoined();
    msg_info("IAB") << "DataRepository paths = " << DataRepository.getPathsJoined();
    msg_info("IAB") << "GuiDataRepository paths = " << GuiDataRepository.getPathsJoined();


    // Initialise paths
    sofa::gui::BaseGUI::setConfigDirectoryPath(SofaBuildPath + "/config", true);
    sofa::gui::BaseGUI::setScreenshotDirectoryPath(SetDirectory::GetCurrentDir() +  "/screenshots", true);

    for (unsigned int i=0; i<plugins.size(); i++)
    {
      // provide full path to IAB plugin in cmd line
      msg_info("SingleIAB") << "Loading " << IABBuildPATH + "/lib/" + plugins[i];
      PluginManager::getInstance().loadPluginByPath(IABBuildPATH + "/lib/" + plugins[i]);
    }

    std::string configPluginPath =  IAB_PATH + "/plugins/plugins.conf";

    if (PluginRepository.findFile(configPluginPath, "", nullptr))
    {
        msg_info("IAB") << "Loading patient plugins in " << configPluginPath;
        PluginManager::getInstance().readFromIniFile(configPluginPath);
    }
    else
        msg_info("IAB") << "Plugins not provided. Not loading plugins.";

    PluginManager::getInstance().init();

    if (int err = GUIManager::Init(argv[0],gui.c_str()))
        return err;

    // if (fileName.empty())
    std::string fileName = DataRepository.getFile(SetDirectory::GetCurrentDir() + "/../scenes/dome_test.scn");

    if (int err=GUIManager::createGUI(nullptr))
        return err;

    //To set a specific resolution for the viewer, use the component ViewerSetting in you scene graph
    GUIManager::SetDimension(width, height);

    // Create and register the SceneCheckerListener before scene loading
    if(!noSceneCheck)
        sofa::simulation::SceneLoader::addListener( SceneCheckerListener::getInstance() );

    /* this contains the states, positions, mass, geometry and such
     see ~/sofa/v19.06/modules/SofaMiscForceField/SofaMiscForceField_test/MeshMatrixMass_test.cpp#L102-111*/
    Node::SPtr groot = sofa::simulation::getSimulation()->load(fileName.c_str());
    if( !groot )
        groot = sofa::simulation::getSimulation()->createNewGraph("");

    if (!verif.empty())
        loadVerificationData(verif, fileName, groot.get());

    sofa::simulation::getSimulation()->init(groot.get());  // similar: modules/SofaMiscForceField/SofaMiscForceField_test/MeshMatrixMass_test.cpp#L133*/
    // see accessing node compos in v19.06/SofaKernel/framework/sofa/simulation/Node.cpp

    GUIManager::SetScene(groot,fileName.c_str(), temporaryFile);
    // get all three nodes
    auto dome_head = groot->getChild("DomeHeadNode"); //root node has no children
    auto dome_ring = groot->getChild("DomeRingNode"); //root node has no children
    auto dome_base = groot->getChild("DomeCoverNode"); //root node has no children
    // get tetrahedrals associated with each node
    // sofa::component::container::MechanicalObject<Vec3d> mechObj;
    auto dome_head_tetras = dome_head->getObject(mechObj, "dofs");
    auto dome_ring_tetras = dome_ring->getObject("dofs");
    auto dome_base_tetras = dome_base->getObject("dofs");
    // test with random simultaneous motion of all tetrahedrals in head and ring
    msg_info("dome_head_tetras") << dome_head_tetras;
    msg_info("dome_base_tetras") << dome_base_tetras;
    // get position, velocity, force vec and rest position coordinates for head for ex.
    auto dome_head_pos = dome_head->getMechanicalState();
    // // get heacontext
    // auto head_ctx = dome_head_tetras->getContext();
    // msg_info("head_ctx ") << head_ctx;
    // auto dome_head_rest_pos = dome_head_tetras->rest_position();
    // auto dome_ring_rest_pos = dome_ring_tetras->rest_position();
    // auto dome_base_rest_pos = dome_base_tetras->rest_position();
    /*
    // // get degrees of freedom
    auto states = groot->getState();
    // // get mechsnical degrees of freedom
    auto mechStates = groot->getMechanicalState();
    auto shader = groot->getShader();
    groot->printComponents();
    // msg_info("SphereDeform") << "states : " << states;
    // msg_info("SphereDeform") << "mechStates : " << mechStates;
    msg_info("SphereDeform") << "states (DoFs): " << states; // get degrees of freedom
    // msg_info("SphereDeform") << "mechStates values: " << mechStates->x.getValue(); // specified by VecXd in scene file
    msg_info("SphereDeform") << "mechStates Dim: " << mechStates->getCoordDimension(); // specified by VecXd in scene file
    msg_info("SphereDeform") << "mechStates Dim Derivs: " << mechStates->getDerivDimension();  // specified by VecXd in scene file
    msg_info("SphereDeform") << "mechStates Template Name: " << mechStates->getTemplateName(); // returns sofa::core::behavior::BaseMechanicalState has no member named ‘readPositions’
*/
    // test random rotation and translation of scene:
      // v19.06/modules/SofaGeneralDeformable/SofaGeneralDeformable_test/StiffSpringForceField_test.cpp#95

    // we can get the indices of particles in the given bounding box:
    // see /Users/olalekanogunmolu/sofa/v19.06/SofaKernel/framework/sofa/core/behavior/MechanicalState.h#L86
    //=======================================
    //Apply Options
    groot->setAnimate(false);
    // test expansion and deformation here for a single soro

    if( computationTimeSampling>0 )
    {
        sofa::helper::AdvancedTimer::setEnabled("Animate", true);
        sofa::helper::AdvancedTimer::setInterval("Animate", computationTimeSampling);
        sofa::helper::AdvancedTimer::setOutputType("Animate", computationTimeOutputType);
    }

    //=======================================
    if(guiViz)
    {
      // Run the main loop
      if (int err = GUIManager::MainLoop(groot,fileName.c_str()))
          return err;
      groot = dynamic_cast<Node*>( GUIManager::CurrentSimulation() );
    }

    if (groot!=NULL)
        sofa::simulation::getSimulation()->unload(groot);

    if (guiViz)
      GUIManager::closeGUI();

    sofa::simulation::common::cleanup();
    sofa::simulation::graph::cleanup();
    return 0;
}

///applications/plugins/SofaTest/Elasticity_test.inl
