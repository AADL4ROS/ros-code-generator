from methods.Method import Method
from datatypes.Type import Void

class TearDown(Method):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.namespace      = self.associated_class.class_name
        self.method_name    = "tearDown"
        self.return_type    = Void( self.associated_class )

        self.codice  = "ROS_INFO(\"Node is shutting down\");\n"
        self.codice += "return;\n"