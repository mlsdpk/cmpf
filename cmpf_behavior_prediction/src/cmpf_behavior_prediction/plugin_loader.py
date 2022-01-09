import sys
import importlib
import roslib
import rospkg

from cmpf_behavior_prediction.factory import _PLUGINS


def create_plugin_instance(plugin_name: str):
    try:
        plugin = _PLUGINS[plugin_name]
        return plugin()
    except KeyError:
        raise ValueError(f"Plugin {plugin_name} not found.")


def load_plugin_modules(default_pkg_name: str = None):

    plugin_module_names = []
    rospack = rospkg.RosPack()

    # find the default plugins
    if default_pkg_name is not None:
        # get default plugin module names
        plugin_module_names.append(get_plugin_module_names(
            rospack, default_pkg_name))

    # find the plugins globally

    # find all ros packages that depends on cmpf_behavior_prediction
    pkgs_to_check = rospack.get_depends_on(
        'cmpf_behavior_prediction', implicit=False)

    # get all the plugins from each package
    for pkg in pkgs_to_check:
        plugin_module_names.append(get_plugin_module_names(rospack, pkg))

    for pkg, plugins in plugin_module_names:
        # load that packages namespace
        roslib.load_manifest(pkg)
        for plugin in plugins:
            try:
                # import the plugin module
                importlib.import_module(plugin)
            except Exception as e:
                print(
                    f"Unable to load plugin {plugin} from package {pkg}. Exception thrown: {e}", file=sys.stderr)


def get_plugin_module_names(rospack, pkg_name):
    pkg_xml_file = rospack.get_manifest(pkg_name)

    # retrieve plugin module names from package.xml
    plugin_module_names = pkg_xml_file.get_export(pkg_name, 'plugin')

    if not plugin_module_names:
        return
    elif len(plugin_module_names) == 0:
        print(
            f"Cannot load plugin {pkg_name}: invalid 'plugin' attribute", file=sys.stderr)
        return

    return (pkg_name, plugin_module_names)
