class BaseBehaviorPredictor(object):
    """Base Behavior Predictor Plugin Class"""

    def __init__(self) -> None:
        pass

    def initialize(self, name: str):
        raise NotImplementedError

    def activate(self):
        raise NotImplementedError
