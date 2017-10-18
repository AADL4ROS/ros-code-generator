from methods.Method import Method
from datatypes.Type import Void
from variables.Variable import Variable
from datatypes.Type import Int
from datatypes.Type import PointerToCharPointer

class Main(Method):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.namespace      = None
        self.method_name    = "main"
        self.return_type    = Int( self.associated_class )

        # int argc
        input_argc = Variable( self.associated_class )
        input_argc.setIsParameter()
        input_argc.setName("argc")
        input_argc.setType( Int( self.associated_class ) )

        # char **argv
        input_argv = Variable(self.associated_class)
        input_argv.setIsParameter()
        input_argv.setName("argv")
        input_argv.setType( PointerToCharPointer(self.associated_class) )

        self.addInputParameter( input_argc )
        self.addInputParameter( input_argv )

        self.addTopCode( "ros::init(argc, argv, {}, ros::init_options::NoSigintHandler);"
                         .format(self.associated_class.node_name))
        self.addTopCode("signal(SIGINT, nodeSigintHandler);")

        self.addBottomCode("return 0;")