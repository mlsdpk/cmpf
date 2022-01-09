#!/usr/bin/env python3/

import rospy
from cmpf_behavior_prediction.plugin_loader import load_plugin_modules, create_plugin_instance
from cmpf_behavior_prediction.base_behavior_predictor import BaseBehaviorPredictor

if __name__ == '__main__':
    rospy.init_node('test_node')

    # load all the plugins
    load_plugin_modules("cmpf_behavior_prediction")

    # later this must be given from ros parameter
    # we will select one of the plugin to use
    plugin_name = 'cmpf_behavior_prediction.plugins.my_test_plugin.my_test_plugin'

    my_plugin: BaseBehaviorPredictor
    try:
        my_plugin = create_plugin_instance(plugin_name)
    except Exception as ex:
        print(ex)

    my_plugin.on_init()
    print("DONE")
