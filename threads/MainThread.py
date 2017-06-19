import logging
log = logging.getLogger("root")

from threads.AADLThread import AADLThread
from threads.AADLThread import AADLThreadType

import threads.AADLThreadFunctionsSupport as tfs

from libraries.Library import ROSBase_ROSNode

from methods.Constructor    import Constructor
from methods.Prepare        import Prepare
from methods.TearDown       import TearDown
from methods.ErrorHandling  import ErrorHandling
from methods.NodeSigintHandler import NodeSigintHandler
from methods.Main           import Main
from constants.Constant     import Constant
from variables.Std_String   import Std_String

###################
### MAIN THREAD ###
###################
class MainThread(AADLThread):
    def __init__(self, _system_root, _process, _thread, _associated_class):
        super().__init__(_system_root, _process, _thread, AADLThreadType.MAIN_THREAD, _associated_class)
        log.info("Main thread {}".format(self.name))

        # Parametri del main thread
        self.prepare        = None
        self.tearDown       = None
        self.errorHandling  = None
        self.constructor    = None

        # Aggiungo la libreria base del nostro nodo ROS
        self.associated_class.addLibrary(ROSBase_ROSNode())

        # Aggiungo i metodi nodeSigintHandler e main
        nodeSigintHandler = NodeSigintHandler( self.associated_class )
        self.associated_class.addPrivateMethod( nodeSigintHandler )

        main = Main( self.associated_class )
        main.addMiddleCode("{} node;".format( self.associated_class.node_name ))
        main.addMiddleCode("node.start();")
        self.associated_class.addPrivateMethod( main )

        # Creo la costante con il nome del nodo
        const_node_name = Constant()
        const_node_name.setName("NODE_NAME")
        const_node_name.setValue(self.associated_class.node_name)
        self.associated_class.addConstant( const_node_name )

        # Creo il parametro con il nome del nodo
        param_node_name = Std_String( self.associated_class )
        param_node_name.setName( "node_name" )
        self.associated_class.addParameter( param_node_name )

        # Creo il metodo costruttore per la classe
        self.constructor = Constructor( self.associated_class )
        self.associated_class.addPublicMethod( self.constructor )

    def populateData(self):
        # Ottengo tutti i dati relativi al main thread
        (prepare, tearDown, errorHandler) = tfs.getMainThreadFunctions( self.thread )

        self.prepare                = Prepare( self.associated_class )
        self.prepare.source_text    = tfs.getSourceText( prepare )

        self.tearDown               = TearDown( self.associated_class )
        self.tearDown.source_text   = tfs.getSourceText( tearDown )

        self.errorHandling              = ErrorHandling( self.associated_class )
        self.errorHandling.source_text  = tfs.getSourceText( errorHandler )

        self.associated_class.addPrivateMethod( self.prepare )
        self.associated_class.addPrivateMethod( self.tearDown )
        self.associated_class.addPrivateMethod( self.errorHandling )

        self.prepare.addTopCode("params.node_name = NODE_NAME;")
        self.prepare.addMiddleCode("handle.getParam(\"node_name\", params.node_name);")
        self.prepare.addBottomCode("return true;")

        return (True, "")