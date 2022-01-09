#!/usr/bin/env python3

import sys
import rospy

from cmpf_behavior_prediction.plugin_loader import load_plugin_modules, create_plugin_instance


class BehaviorPredictionServer(object):

    def __init__(self, name):
        self._name = name

        # load all the plugins
        load_plugin_modules("cmpf_behavior_prediction")

        # get ros parameters

        self._prediction_frequency = rospy.get_param(
            "~prediction_frequency", default=5.0)

        # retrieve plugin from the parameter server
        if (rospy.has_param("~plugin")):
            self._plugin_id = rospy.get_param("~plugin")
        else:
            rospy.logerr(
                f"[cmpf_behavior_prediction] ROS parameter 'plugin' not found in the server.")
            sys.exit()

        # retrieve plugin name from the parameter server
        if (rospy.has_param("~"+self._plugin_id+"/plugin")):
            self._plugin_name = rospy.get_param("~"+self._plugin_id+"/plugin")
        else:
            rospy.logerr(
                f"[cmpf_behavior_prediction] plugin parameter not defined for {self._plugin_id}")
            sys.exit()

        # create the plugin
        try:
            self._plugin_obj = create_plugin_instance(self._plugin_name)
        except Exception as ex:
            rospy.logerr(
                f"[cmpf_behavior_prediction] Error occured while creating a plugin. Exception: {ex}")
            sys.exit()

        # initialize the plugin
        self._plugin_obj.initialize(name=self._plugin_id)

        # create timer and activate the plugin
        rospy.Timer(rospy.Duration(1.0/self._prediction_frequency),
                    self.prediction_loop)

    def prediction_loop(self, event=None):
        self._plugin_obj.activate()


if __name__ == '__main__':
    rospy.init_node('behavior_prediction_server')
    server = BehaviorPredictionServer(rospy.get_name())
    rospy.spin()
