from cmpf_behavior_prediction.base_behavior_predictor import BaseBehaviorPredictor
from cmpf_behavior_prediction import factory


@factory.register
class MyTestPlugin(BaseBehaviorPredictor):
    def __init__(self):
        super().__init__()

    def on_init(self):
        print("On Init")

    def on_active(self):
        print("On Active")
