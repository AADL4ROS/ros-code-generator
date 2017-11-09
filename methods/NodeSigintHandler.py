from methods.Method import Method
from datatypes.Type import Void
from variables.Variable import Variable
from datatypes.Type import Int

class NodeSigintHandler(Method):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.namespace      = None
        self.method_name    = "nodeSigintHandler"
        self.return_type    = Void( self.associated_class )

        input_sig = Variable( self.associated_class )
        input_sig.setIsParameter()
        input_sig.setName("sig")
        input_sig.setType( Int( self.associated_class ) )

        self.addInputParameter( input_sig )

        self.addTopCode( "g_request_shutdown = 1;")