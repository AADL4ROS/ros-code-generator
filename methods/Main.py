from methods.Method import Method
from datatypes.Type import Void
from variables.Variable import Variable
from datatypes.Type import Int
from datatypes.Type import Char

class Main(Method):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.namespace      = None
        self.method_name    = "main"
        self.return_type    = Int( self.associated_class )

        # int argc
        input_argc = Variable( self.associated_class )
        input_argc.setName("argc")
        input_argc.setType( Int( self.associated_class ) )

        # char **argv
        input_argv = Variable(self.associated_class)
        input_argv.setName("**argv")
        input_argv.setType( Char(self.associated_class) )

        self.addInputParameter( input_argc )
        self.addInputParameter( input_argv )

        self.addTopCode( "ros::init(argc, argv, NODE_NAME, ros::init_options::NoSigintHandler);");
        self.addTopCode( "while(!ros::master::check())\n"
                            "\tusleep(1000);");
        self.addTopCode("signal(SIGINT, nodeSigintHandler);");

        self.addBottomCode("return 0;")