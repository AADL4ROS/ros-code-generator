class CObject():
    def __init__(self, _associated_class):
        self.associated_class = _associated_class

    def isEqualTo(self, another_object):
        raise NotImplementedError("This method must be implemented by every subclass")

    def generateCode(self):
        raise NotImplementedError("This method must be implemented by every subclass")