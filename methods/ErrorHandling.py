from methods.Method import Method
from datatypes.Type import Void

class ErrorHandling(Method):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.namespace      = self.associated_class.class_name
        self.method_name    = "errorHandling"
        self.return_type    = Void( self.associated_class )

        # @TODO: Codice provvisorio
        self.codice  = "ROSNode::errorHandling();\n"