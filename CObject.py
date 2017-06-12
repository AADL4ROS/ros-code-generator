class CObject():
    def __init__(self, _associated_class):
        self.associated_class = _associated_class

    def generateCode(self):
        raise NotImplementedError("This method must be implemented by every subclass")