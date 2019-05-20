#include <sstream>
using std::ostringstream ;
#include <fstream>

#include <string>
using std::string;

#include <vector>
using std::vector;


#include <sofa/helper/ArgumentParser.h>
#include <SofaSimulationCommon/common.h>
#include <sofa/simulation/Node.h>
#include <sofa/helper/system/PluginManager.h>
#include <sofa/simulation/config.h> // #defines SOFA_HAVE_DAG (or not)
#include <SofaSimulationCommon/init.h>
#ifdef SOFA_HAVE_DAG
#include <SofaSimulationGraph/init.h>
#include <SofaSimulationGraph/DAGSimulation.h>
#endif
#include <SofaSimulationTree/init.h>
#include <SofaSimulationTree/TreeSimulation.h>
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
#include <sofa/gui/BatchGUI.h>  // For the default number of iterations
#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/atomic.h>

using sofa::core::ExecParams ;

#include <sofa/helper/system/console.h>
using sofa::helper::Utils;

using sofa::component::misc::CompareStateCreator;
using sofa::component::misc::ReadStateActivator;
using sofa::simulation::tree::TreeSimulation;
using sofa::simulation::graph::DAGSimulation;
using sofa::helper::system::SetDirectory;
using sofa::core::objectmodel::BaseNode ;
using sofa::gui::BatchGUI;
using sofa::gui::BaseGUI;

#include <sofa/helper/logging/Messaging.h>

#include <sofa/helper/logging/ConsoleMessageHandler.h>
using sofa::helper::logging::ConsoleMessageHandler ;

#include <sofa/core/logging/RichConsoleStyleMessageFormatter.h>
using  sofa::helper::logging::RichConsoleStyleMessageFormatter ;

#include <sofa/core/logging/PerComponentLoggingMessageHandler.h>
using  sofa::helper::logging::MainPerComponentLoggingMessageHandler ;

#include <sofa/helper/AdvancedTimer.h>

#ifdef WIN32
#include <windows.h>
#endif

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <sofa/gui/GuiDataRepository.h>
using sofa::gui::GuiDataRepository ;

using sofa::helper::system::DataRepository;
using sofa::helper::system::PluginRepository;
using sofa::helper::system::PluginManager;

#include <sofa/helper/logging/MessageDispatcher.h>
using sofa::helper::logging::MessageDispatcher ;

#include <sofa/helper/logging/ClangMessageHandler.h>
using sofa::helper::logging::ClangMessageHandler ;

#include <sofa/helper/logging/ExceptionMessageHandler.h>
using sofa::helper::logging::ExceptionMessageHandler;

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)


void loadVerificationData(string& directory, string& filename, Node* node)
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

bool pathToSofaBuild(boost::filesystem::path&& SofaInstallPath,
                     boost::filesystem::path&& SofaBuildPath)
{
  // set sofa build path
  const std::string cwd = SetDirectory::GetCurrentDir();
  // msg_info(" cwd: ") << cwd;

  // SofaInstallPath = cwd + "/../../../../../sofa/master/build/install/";
  // SofaBuildPath   = cwd + "/../../../../../sofa/master/build/";

  SofaInstallPath = "/Users/olalekanogunmolu/sofa/master/build/install/";
  SofaBuildPath   = "/Users/olalekanogunmolu/sofa/master/build/";

  return true;
}

// ---------------------------------------------------------------------
// ---
// ---------------------------------------------------------------------
int main(int argc, char** argv)
{
    boost::filesystem::path SofaInstallPath, SofaBuildPath;
    if(!(pathToSofaBuild(std::move(SofaInstallPath),
                         std::move(SofaBuildPath))
                       )){
            msg_info(" ") << "could not load the paths";
                       }

    // Add resources dir to GuiDataRepository
    auto runSofaIniFilePath = SofaInstallPath / "share/sofa/gui/qt";
    auto dir = SofaInstallPath / "share/sofa/gui/qt/";
    if(FileSystem::isDirectory(dir.string()))
    {
        sofa::gui::GuiDataRepository.addFirstPath(dir.string());
    }

    msg_info("dir ") << dir.string() ;
    // msg_info("SofaInstallPath ") << SofaInstallPath.string() ;

    // Force add plugins dir to PluginRepository (even if not existing)
    PluginRepository.addFirstPath(SofaBuildPath.string() + "/plugins");
    PluginRepository.addFirstPath(SofaBuildPath.string() + "/lib");

    sofa::helper::BackTrace::autodump();

    ExecParams::defaultInstance()->setAspectID(0);

    sofa::gui::initMain();

    string fileName ;
    bool        startAnim = false;
    bool        showHelp = false;
    bool        printFactory = false;
    bool        loadRecent = false;
    bool        temporaryFile = false;
    bool        testMode = false;
    bool        noAutoloadPlugins = false;
    bool        noSceneCheck = false;
    unsigned int nbMSSASamples = 1;
    bool computationTimeAtBegin = false;
    unsigned int computationTimeSampling=0; ///< Frequency of display of the computation time statistics, in number of animation steps. 0 means never.
    string    computationTimeOutputType="stdout";

    string gui = "";
    string verif = "";

#if defined(SOFA_HAVE_DAG)
    string simulationType = "dag";
#else
    string simulationType = "tree";
#endif

    vector<string> plugins;
    vector<string> files;
#ifdef SOFA_SMP
    string nProcs="";
    bool        disableStealing = false;
    bool        affinity = false;
#endif
    string colorsStatus = "unset";
    string messageHandler = "auto";
    bool enableInteraction = false ;
    int width = 800;
    int height = 600;

    string gui_help = "choose the UI (";
    gui_help += GUIManager::ListSupportedGUI('|');
    gui_help += ")";

    ArgumentParser* argParser = new ArgumentParser(argc, argv);
    argParser->addArgument(po::value<bool>(&showHelp)->default_value(false)->implicit_value(true),                  "help,h", "Display this help message");
    argParser->addArgument(po::value<bool>(&startAnim)->default_value(false)->implicit_value(true),                 "start,a", "start the animation loop");
    argParser->addArgument(po::value<bool>(&computationTimeAtBegin)->default_value(false)->implicit_value(true),    "computationTimeAtBegin,b", "Output computation time statistics of the init (at the begin of the simulation)");
    argParser->addArgument(po::value<unsigned int>(&computationTimeSampling)->default_value(0),                     "computationTimeSampling", "Frequency of display of the computation time statistics, in number of animation steps. 0 means never.");
    argParser->addArgument(po::value<std::string>(&computationTimeOutputType)->default_value("stdout"),             "computationTimeOutputType,o", "Output type for the computation time statistics: either stdout, json or ljson");
    argParser->addArgument(po::value<std::string>(&gui)->default_value(""),                                         "gui,g", gui_help.c_str());
    argParser->addArgument(po::value<std::vector<std::string>>(&plugins),                                           "load,l", "load given plugins");
    argParser->addArgument(po::value<bool>(&noAutoloadPlugins)->default_value(false)->implicit_value(true),         "noautoload", "disable plugins autoloading");
    argParser->addArgument(po::value<bool>(&noSceneCheck)->default_value(false)->implicit_value(true),              "noscenecheck", "disable scene checking for each scene loading");

    // example of an option using lambda function which ensure the value passed is > 0
    argParser->addArgument(po::value<unsigned int>(&nbMSSASamples)->default_value(1)->notifier([](unsigned int value)
    {
        if (value < 1) {
            std::cerr << "msaa sample cannot be lower than 1" << std::endl;
            exit( EXIT_FAILURE );
        }
    }),                                                                                                             "msaa,m", "number of samples for MSAA (Multi Sampling Anti Aliasing ; value < 2 means disabled");

    argParser->addArgument(po::value<bool>(&printFactory)->default_value(false)->implicit_value(true),              "factory,p", "print factory logs");
    argParser->addArgument(po::value<bool>(&loadRecent)->default_value(false)->implicit_value(true),                "recent,r", "load most recently opened file");
    argParser->addArgument(po::value<std::string>(&simulationType),                                                 "simu,s", "select the type of simulation (bgl, dag, tree)");
    argParser->addArgument(po::value<bool>(&temporaryFile)->default_value(false)->implicit_value(true),             "tmp", "the loaded scene won't appear in history of opened files");
    argParser->addArgument(po::value<bool>(&testMode)->default_value(false)->implicit_value(true),                  "test", "select test mode with xml output after N iteration");
    argParser->addArgument(po::value<std::string>(&verif)->default_value(""), "verification,v",                     "load verification data for the scene");
    argParser->addArgument(po::value<std::string>(&colorsStatus)->default_value("unset", "auto")->implicit_value("yes"),     "colors,c", "use colors on stdout and stderr (yes, no, auto)");
    argParser->addArgument(po::value<std::string>(&messageHandler)->default_value("auto"), "formatting,f",          "select the message formatting to use (auto, clang, sofa, rich, test)");
    argParser->addArgument(po::value<bool>(&enableInteraction)->default_value(false)->implicit_value(true),         "interactive,i", "enable interactive mode for the GUI which includes idle and mouse events (EXPERIMENTAL)");
    argParser->addArgument(po::value<std::vector<std::string> >()->multitoken(), "argv",                            "forward extra args to the python interpreter");

#ifdef SOFA_SMP
    argParser->addArgument(po::value<bool>(&disableStealing)->default_value(false)->implicit_value(true),           "disableStealing,w", "Disable Work Stealing")
    argParser->addArgument(po::value<std::string>(&nProcs)->default_value(""),                                      "nprocs", "Number of processor")
    argParser->addArgument(po::value<bool>(&affinity)->default_value(false)->implicit_value(true),                  "affinity", "Enable aFfinity base Work Stealing")
#endif

    addGUIParameters(argParser);
    argParser->parse();
    files = argParser->getInputFileList();

    if(showHelp)
    {
        argParser->showHelp();
        exit( EXIT_SUCCESS );
    }

    // Note that initializations must be done after ArgumentParser that can exit the application (without cleanup)
    // even if everything is ok e.g. asking for help
    sofa::simulation::tree::init();
#ifdef SOFA_HAVE_DAG
    sofa::simulation::graph::init();
#endif
    sofa::component::initComponentBase();
    sofa::component::initComponentCommon();
    sofa::component::initComponentGeneral();
    sofa::component::initComponentAdvanced();
    sofa::component::initComponentMisc();

#ifdef SOFA_HAVE_DAG
    if (simulationType == "tree")
        sofa::simulation::setSimulation(new TreeSimulation());
    else
        sofa::simulation::setSimulation(new DAGSimulation());
#else //SOFA_HAVE_DAG
    sofa::simulation::setSimulation(new TreeSimulation());
#endif

    if (colorsStatus == "unset") {
        // If the parameter is unset, check the environment variable
        const char * colorStatusEnvironment = std::getenv("SOFA_COLOR_TERMINAL");
        if (colorStatusEnvironment != nullptr) {
            const std::string status (colorStatusEnvironment);
            if (status == "yes" || status == "on" || status == "always")
                sofa::helper::console::setStatus(sofa::helper::console::Status::On);
            else if (status == "no" || status == "off" || status == "never")
                sofa::helper::console::setStatus(sofa::helper::console::Status::Off);
            else
                sofa::helper::console::setStatus(sofa::helper::console::Status::Auto);
        }
    } else if (colorsStatus == "auto")
        sofa::helper::console::setStatus(sofa::helper::console::Status::Auto);
    else if (colorsStatus == "yes")
        sofa::helper::console::setStatus(sofa::helper::console::Status::On);
    else if (colorsStatus == "no")
        sofa::helper::console::setStatus(sofa::helper::console::Status::Off);

    //TODO(dmarchal): Use smart pointer there to avoid memory leaks !!
    if (messageHandler == "auto" )
    {
        MessageDispatcher::clearHandlers() ;
        MessageDispatcher::addHandler( new ConsoleMessageHandler() ) ;
    }
    else if (messageHandler == "clang")
    {
        MessageDispatcher::clearHandlers() ;
        MessageDispatcher::addHandler( new ClangMessageHandler() ) ;
    }
    else if (messageHandler == "sofa")
    {
        MessageDispatcher::clearHandlers() ;
        MessageDispatcher::addHandler( new ConsoleMessageHandler() ) ;
    }
    else if (messageHandler == "rich")
    {
        MessageDispatcher::clearHandlers() ;
        MessageDispatcher::addHandler( new ConsoleMessageHandler(&RichConsoleStyleMessageFormatter::getInstance()) ) ;
    }
    else if (messageHandler == "test"){
        MessageDispatcher::addHandler( new ExceptionMessageHandler() ) ;
    }
    else{
        msg_warning("") << "Invalid argument '" << messageHandler << "' for '--formatting'";
    }
    MessageDispatcher::addHandler(&MainPerComponentLoggingMessageHandler::getInstance()) ;

    // Output FileRepositories
    msg_info("IAB") << "PluginRepository paths = " << PluginRepository.getPathsJoined();
    msg_info("IAB") << "DataRepository paths = " << DataRepository.getPathsJoined();
    msg_info("IAB") << "GuiDataRepository paths = " << GuiDataRepository.getPathsJoined();

    // Initialise paths
    BaseGUI::setConfigDirectoryPath(SofaBuildPath.string() + "/config", true);
    BaseGUI::setScreenshotDirectoryPath(SofaBuildPath.string()  + "/screenshots", true);

    if (!files.empty())
        fileName = files[0];

    for (unsigned int i=0; i<plugins.size(); i++)
    {
      // std::cout << " plugins [" << i << "] " << plugins[i] << "\n";
      PluginManager::getInstance().loadPlugin(plugins[i]);
    }

    std::string configPluginPath = TOSTRING(CONFIG_PLUGIN_FILENAME);
    std::string defaultConfigPluginPath = TOSTRING(DEFAULT_CONFIG_PLUGIN_FILENAME);

    if (!noAutoloadPlugins)
    {
        if (PluginRepository.findFile(configPluginPath, "", nullptr))
        {
            msg_info("IAB") << "Loading plugin list in " << configPluginPath;
            PluginManager::getInstance().readFromIniFile(configPluginPath);
        }
        else if (PluginRepository.findFile(defaultConfigPluginPath, "", nullptr))
        {
            msg_info("IAB") << "Loading default plugin list in " << defaultConfigPluginPath;
            PluginManager::getInstance().readFromIniFile(defaultConfigPluginPath);
        }
        else
            msg_info("IAB") << "No plugin list found. No plugin will be automatically loaded.";
    }
    else
        msg_info("IAB") << "Automatic plugin loading disabled.";

    PluginManager::getInstance().init();

    if (int err = GUIManager::Init(argv[0],gui.c_str()))
        return err;

    const std::string cwd = SetDirectory::GetCurrentDir();
    
    if (fileName.empty())
    {
        if (loadRecent) // try to reload the latest scene
        {
            string scenes = BaseGUI::getConfigDirectoryPath() + "/runSofa.ini";
            std::ifstream mrulist(scenes.c_str());
            std::getline(mrulist,fileName);
            mrulist.close();
        }
        else
            // fileName = SofaInstallPath.string()+ "/share/sofa/examples/Demos/caduceus.scn";
            fileName = cwd + "../"

        fileName = DataRepository.getFile(fileName);
    }


    if (int err=GUIManager::createGUI(nullptr))
        return err;

    //To set a specific resolution for the viewer, use the component ViewerSetting in you scene graph
    GUIManager::SetDimension(width, height);

    // Create and register the SceneCheckerListener before scene loading
    if(!noSceneCheck)
    {
        sofa::simulation::SceneLoader::addListener( SceneCheckerListener::getInstance() );
    }

    Node::SPtr groot = sofa::simulation::getSimulation()->load(fileName.c_str());
    if( !groot )
        groot = sofa::simulation::getSimulation()->createNewGraph("");

    if (!verif.empty())
    {
        loadVerificationData(verif, fileName, groot.get());
    }

    if( computationTimeAtBegin )
    {
        sofa::helper::AdvancedTimer::setEnabled("Init", true);
        sofa::helper::AdvancedTimer::setInterval("Init", 1);
        sofa::helper::AdvancedTimer::setOutputType("Init", computationTimeOutputType);
        sofa::helper::AdvancedTimer::begin("Init");
    }

    sofa::simulation::getSimulation()->init(groot.get());
    if( computationTimeAtBegin )
    {
        msg_info("") << sofa::helper::AdvancedTimer::end("Init", groot.get());
    }
    GUIManager::SetScene(groot,fileName.c_str(), temporaryFile);


    //=======================================
    //Apply Options

    if (startAnim)
        groot->setAnimate(true);
    if (printFactory)
    {
        msg_info("") << "////////// FACTORY //////////" ;
        sofa::helper::printFactoryLog();
        msg_info("") << "//////// END FACTORY ////////" ;
    }

    if( computationTimeSampling>0 )
    {
        sofa::helper::AdvancedTimer::setEnabled("Animate", true);
        sofa::helper::AdvancedTimer::setInterval("Animate", computationTimeSampling);
        sofa::helper::AdvancedTimer::setOutputType("Animate", computationTimeOutputType);
    }

    //=======================================
    // Run the main loop
    if (int err = GUIManager::MainLoop(groot,fileName.c_str()))
        return err;
    groot = dynamic_cast<Node*>( GUIManager::CurrentSimulation() );

    if (testMode)
    {
        string xmlname = fileName.substr(0,fileName.length()-4)+"-scene.scn";
        msg_info("") << "Exporting to XML " << xmlname ;
        sofa::simulation::getSimulation()->exportXML(groot.get(), xmlname.c_str());
    }

    if (groot!=NULL)
        sofa::simulation::getSimulation()->unload(groot);


    GUIManager::closeGUI();

    sofa::simulation::common::cleanup();
    sofa::simulation::tree::cleanup();
#ifdef SOFA_HAVE_DAG
    sofa::simulation::graph::cleanup();
#endif
    return 0;
}
