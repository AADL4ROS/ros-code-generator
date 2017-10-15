import logging
log = logging.getLogger("root")

from threads.AADLThread import AADLThread

import threads.AADLThreadFunctionsSupport as tfs

from libraries.Library import ROSBase_ROSNode

from methods.Constructor    import Constructor
from methods.Prepare        import Prepare
from methods.TearDown       import TearDown
from methods.ErrorHandling  import ErrorHandling
from methods.NodeSigintHandler import NodeSigintHandler
from methods.Main           import Main
from constants.Constant     import Constant
from asn1tools.parser import parse_file

import os
from datatypes.DatatypeConversion import getROSDatatypeFromASN1
from variables.Variable import Variable

from datatypes.Type import String as StdString
import re

###################
### MAIN THREAD ###
###################
class MainThread(AADLThread):
    def __init__(self, _system_root, _process, _thread, _associated_class):
        super().__init__(_system_root, _process, _thread, _associated_class)
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

        # Creo il metodo costruttore per la classe
        self.constructor = Constructor( self.associated_class )
        self.associated_class.addPublicMethod( self.constructor )

    #####################
    ### GET ASN STATE ###
    #####################

    STATE_ASN_DEFINITION    = "State"
    STATE_ASN_PARAMS        = "Params"
    STATE_ASN_VARS          = "Vars"

    # Ogni process/node ha un file che descrive params e vars, questo metodo si occupa
    # di processarlo ed aggiungere la variabili e parametri corretti
    def getStateASN(self):
        parameters = []
        variables = []

        # @TODO: per ora è assegnato al process come Source_Text, vedere se è da cambiare
        asn_file = tfs.getSourceText(self.process)

        if asn_file is None:
            log.warning("No Params and Vars ASN.1 file specified for {}".format(self.associated_class.node_name))
            return (parameters, variables)

        # @TODO: il percorso del file AADL è lo stesso del file XML
        ocarina_ros_path = "../ocarina-ros/"
        parsed_asn = parse_file(os.path.join(ocarina_ros_path, asn_file))

        # Estraggo tutti i tipi che NON sono Params o Vars
        custom_types = []
        for t in parsed_asn[self.STATE_ASN_DEFINITION]['types']:
            if t != self.STATE_ASN_PARAMS and \
                t != self.STATE_ASN_VARS:
                custom_types.append(t)

        params_asn  = parsed_asn[self.STATE_ASN_DEFINITION]['types'][self.STATE_ASN_PARAMS]['members']
        vars_asn    = parsed_asn[self.STATE_ASN_DEFINITION]['types'][self.STATE_ASN_VARS]['members']

        # RegEx per controllare se una stringa inizia e finisce con le virgolette
        string_apex_regex = re.compile("\"(.+)\"")

        for index, p_v in enumerate(params_asn + vars_asn):
            var_type    = p_v['type']
            var_name    = p_v['name']
            if 'default' in p_v:
                default_val = p_v['default']
                # Rimuovo i caratteri di escape
                default_val = str(default_val).replace("\\", "")
            else:
                default_val = None

            tmp_param = Variable(self)
            tmp_param.setType(getROSDatatypeFromASN1(var_type, self.associated_class))
            tmp_param.setName(var_name)

            # @TODO: Non si può fare nelle struct, verrà aggiunto in CFG
            #tmp_param.setDefaultValue(default_val)

            if index < len(params_asn):
                parameters.append(tmp_param)
            else:
                variables.append(tmp_param)

        return (parameters, variables)

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

        # Ottengo lo stato (parameters, variables) in ASN.1 del nodo
        (parameters, variables) = self.getStateASN()

        for p in parameters:
            self.associated_class.addParameter(p)

            if p.hasDefaultValue():
                self.prepare.addMiddleCode("handle.param<{}>(\"{}\", params.{}, {});"
                                           .format(p.type.generateCode(),
                                                   p.name,
                                                   p.name,
                                                   p.default_value))
            else:
                self.prepare.addMiddleCode("handle.getParam(\"{}\", params.{});"
                                           .format(p.name, p.name))
            log.info("Parameters: {}".format(p.generateCode()) )

        for v in variables:
            self.associated_class.addVariable(v)

        self.prepare.addBottomCode("return true;")

        return (True, "")

