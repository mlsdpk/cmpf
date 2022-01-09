import rospy

from cmpf_behavior_prediction.base_behavior_predictor import BaseBehaviorPredictor
from cmpf_behavior_prediction import factory


@factory.register
class MyTestPlugin(BaseBehaviorPredictor):
    def __init__(self):
        super().__init__()

    def initialize(self, name: str):
        print(f"On Init: {name}")

    def activate(self):
        print("On Active")
