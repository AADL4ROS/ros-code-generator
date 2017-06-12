import logging
log = logging.getLogger("root")

from threads.AADLThread import AADLThread
from threads.AADLThread import AADLThreadType

import threads.AADLThreadFunctionsSupport as tfs

from methods.Prepare        import Prepare
from methods.TearDown       import TearDown
from methods.ErrorHandling  import ErrorHandling
from constants.Constant     import Constant

###################
### MAIN THREAD ###
###################
class MainThread(AADLThread):
    def __init__(self, _process, _thread, _associated_class):
        super().__init__(_process, _thread, AADLThreadType.MAIN_THREAD, _associated_class)

        # Parametri del main thread
        self.prepare        = None
        self.tearDown       = None
        self.errorHandling  = None

        log.info( "Main thread {}".format( self.name ) )

        const_node_name = Constant()
        const_node_name.setName("NODE_NAME")
        const_node_name.setValue(self.associated_class.node_name)
        self.associated_class.addConstant( const_node_name )

        self.populateData()

    def populateData(self):
        # Ottengo tutti i dati relativi al main thread
        (prepare, tearDown, errorHandler) = tfs.getMainThreadFunctions( self.thread )

        self.prepare                = Prepare( self.associated_class )
        self.prepare.source_text    = tfs.getSourceText( prepare )

        self.tearDown               = TearDown(self.associated_class)
        self.tearDown.source_text   = tfs.getSourceText( tearDown )

        self.errorHandling              = ErrorHandling(self.associated_class)
        self.errorHandling.source_text  = tfs.getSourceText( errorHandler )

        self.associated_class.addPrivateMethod( self.prepare )
        self.associated_class.addPrivateMethod( self.tearDown )
        self.associated_class.addPrivateMethod( self.errorHandling )