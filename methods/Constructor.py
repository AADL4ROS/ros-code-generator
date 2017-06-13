from methods.Method import Method
from datatypes.Type import Nothing

class Constructor(Method):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.namespace      = self.associated_class.class_name
        self.method_name    = self.associated_class.class_name
        self.return_type    = Nothing( self.associated_class )

        self.addTopCode( "setName(NODE_NAME);" )