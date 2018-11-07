from methods.Method import Method
from datatypes.Type import Bool


class Prepare(Method):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.namespace = self.associated_class.class_name
        self.method_name = "prepare"
        self.return_type = Bool(self.associated_class)
