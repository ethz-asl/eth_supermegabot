#include "maplab-console/maplab-console.h"

#include <dlfcn.h>

#include <cstdlib>
#include <fstream>  // NOLINT
#include <string>
#include <unordered_set>

#include <gflags/gflags.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/string-tools.h>
#include <visualization/rviz-visualization-sink.h>

DEFINE_bool(ros_free, false, "Enable this flag to run on systems without ROS");

namespace maplab {

MapLabConsole::MapLabConsole(
    const std::string& console_name, int argc, char** argv)
    : common::Console(console_name) {
  setSelectedMapKey("");
  discoverAndInstallPlugins(argc, argv);
}

MapLabConsole::~MapLabConsole() {
  uninstallAllPlugins();
  for (void*& plugin_handle : plugin_handles_) {
    CHECK_NOTNULL(plugin_handle);
    CHECK_EQ(dlclose(plugin_handle), 0);
  }
}

void MapLabConsole::discoverAndInstallPlugins(int argc, char** argv) {
  // Plotter can be nullptr if --ros_free has been specified.
  const char* plugin_list = std::getenv("MAPLAB_CONSOLE_PLUGINS");
  CHECK(plugin_list != nullptr && plugin_list[0] != '\0')
      << "$MAPLAB_CONSOLE_PLUGINS isn't defined. Please source your workspace.";

  // Find plugins to install.
  std::vector<std::string> plugin_list_vector;

  constexpr char kDelimiter = ';';
  constexpr bool kRemoveEmpty = true;
  common::tokenizeString(
      plugin_list, kDelimiter, kRemoveEmpty, &plugin_list_vector);
  if (plugin_list_vector.empty()) {
    LOG(WARNING) << "No plugin can be loaded as no plugins were definded in "
                 << "the environment variable $MAPLAB_CONSOLE_PLUGINS. Make "
                 << "sure that you built a plugin and that your workspace is "
                 << "sourced.";
  }

  // Dynamically load found plugins.
  std::vector<std::pair</*plugin_handle=*/void*, /*plugin_file=*/std::string>>
      try_load_plugins_handle;
  for (const std::string& plugin_name : plugin_list_vector) {
    void* handle = dlopen(plugin_name.c_str(), RTLD_LAZY);
    if (handle == nullptr) {
      LOG(ERROR) << "Failed to load library " << plugin_name
                 << ". Error message: " << dlerror();
      LOG(ERROR) << "The plugin may not be installed properly. Please try to "
                 << "reinstall the plugin. If the plugin comes from a catkin "
                 << "package, run\n"
                 << "\tcatkin build --no-deps --force-cmake <plugin_package>\n"
                 << "and then try again.";
      continue;
    }
    try_load_plugins_handle.emplace_back(handle, plugin_name);
  }

  // Now that all plugins are loaded we can parse the flags and add them to the
  // completion index.
  google::ParseCommandLineFlags(&argc, &argv, true);
  addAllGFlagsToCompletion();

  if (!FLAGS_ros_free) {
    visualization::RVizVisualizationSink::init();
    plotter_.reset(new visualization::ViwlsGraphRvizPlotter);
    LOG(INFO) << "RVIZ visualization initialized!";
  }

  for (const std::pair<void*, std::string>& handle_lib :
       try_load_plugins_handle) {
    void* handle = handle_lib.first;
    common::PluginCreateFunction create_function =
        common::PluginCreateFunction(dlsym(handle, "createConsolePlugin"));
    common::PluginDestroyFunction destroy_function =
        common::PluginDestroyFunction(dlsym(handle, "destroyConsolePlugin"));
    if (create_function == nullptr || destroy_function == nullptr) {
      LOG(ERROR) << "Error loading the functions from plugin "
                 << handle_lib.second << ". Error message: " << dlerror()
                 << "\nMake sure that your plugin implements the functions "
                 << "\"ConsolePluginBase* "
                 << "createConsolePlugin(common::Console*, "
                 << "visualization::ViwlsGraphRvizPlotter)\" and \"void "
                 << "destroyConsolePlugin(common::ConsolePluginBase*)";
      CHECK_EQ(dlclose(handle), 0);
      continue;
    }
    plugin_handles_.emplace_back(handle);

    // Create and install plugin.
    common::ConsolePluginPtr plugin(
        create_function(this, plotter_.get()), destroy_function);
    VLOG(1) << "Installed plugin " << plugin->getPluginId() << " from "
            << handle_lib.second << ".";
    installPlugin(std::move(plugin));
  }
}

}  // namespace maplab
