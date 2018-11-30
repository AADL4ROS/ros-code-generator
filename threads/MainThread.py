import logging

from threads.AADLThread import AADLThread

import threads.AADLThreadFunctionsSupport as tfs

from libraries.Library import ROSBase_ROSNode

from methods.Constructor import Constructor
from methods.Prepare import Prepare
from methods.TearDown import TearDown
from methods.ErrorHandling import ErrorHandling
from methods.NodeSigintHandler import NodeSigintHandler
from methods.Main import Main
from asn1tools.parser import parse_file

import os
from datatypes.DatatypeConversion import getROSDatatypeFromASN1
from datatypes.Type import Type, String, Bool
from variables.Variable import Variable

from structs.ParametersStruct import ParametersStruct
from structs.VariablesStruct import VariablesStruct
from includes.NodeConfiguration import NodeConfiguration

from structs.Struct import Struct

import global_filepath

# from jsonschema import validate, exceptions
import jsonschema
import json
import sys

log = logging.getLogger("root")


###################
### MAIN THREAD ###
###################
class MainThread(AADLThread):
    def __init__(self, _system_root, _process, _thread, _associated_class):
        super().__init__(_system_root, _process, _thread, _associated_class)
        log.info("Main thread {}".format(self.name))

        # Parametri del main thread
        self.prepare = None
        self.tearDown = None
        self.errorHandling = None
        self.constructor = None

        # Aggiungo la libreria base del nostro nodo ROS
        self.associated_class.addLibrary(ROSBase_ROSNode())

        # Aggiungo i metodi nodeSigintHandler e main
        nodeSigintHandler = NodeSigintHandler(self.associated_class)
        self.associated_class.addPrivateMethod(nodeSigintHandler)

        main = Main(self.associated_class)
        main.addMiddleCode("{} node;".format(self.associated_class.node_name))
        main.addMiddleCode("node.start();")
        self.associated_class.addPrivateMethod(main)

        # Creo la costante con il nome del nodo
        # const_node_name = Constant()
        # const_node_name.setName("NODE_NAME")
        # const_node_name.setValue(self.associated_class.node_name)
        # self.associated_class.addConstant( const_node_name )

        # Creo il metodo costruttore per la classe
        self.constructor = Constructor(self.associated_class)
        self.associated_class.addPublicMethod(self.constructor)

    #################
    # GET ASN STATE #
    #################

    STATE_ASN_DEFINITION = "InternalState"
    STATE_ASN_PARAMS = "Parameters"
    STATE_ASN_VARS = "Variables"

    def getVariableFromASN1(self, p_v):
        var_type = p_v['type']
        var_name = p_v['name']

        if 'default' in p_v:
            default_val = p_v['default']
            # Rimuovo i caratteri di escape
            default_val = str(default_val).replace("\\", "")
        else:
            default_val = None

        # Se mi trovo davanti ad un set, genero una struct con il suo contenuto
        if var_type.upper() == "SET":

            tmp_struct = Struct(self.associated_class, "{}_t".format(var_name))
            tmp_struct.createInstanceWithName(var_name)

            struct_members = p_v['members']
            for m in struct_members:
                tmp_param = self.getVariableFromASN1(m)
                tmp_struct.addVariable(tmp_param)

            return tmp_struct

        else:
            # Se ho una variabile semplice, aggiungo la variabile
            tmp_param = Variable(self)
            tmp_param.setType(getROSDatatypeFromASN1(var_type, self.associated_class))

            if 'tag' in p_v:
                p_v_tags = p_v['tag']
                if 'number' in p_v_tags:
                    var_name = "{}[{}]".format(var_name, p_v_tags['number'])

            tmp_param.setName(var_name)

            if isinstance(tmp_param.type, String) and default_val:
                default_val = "\"{}\"".format(default_val)

            if isinstance(tmp_param.type, Bool) and default_val:
                default_val = default_val.lower()

            tmp_param.setDefaultValue(default_val)

            return tmp_param

    # TODO Work from here - Z.
    def getStateJSON(self):
        parameters = []
        variables = []

        # get the name of the JSON file
        json_file = tfs.getSourceText(self.process)
        json_schema = tfs.getInternalStateSourceText(self.process)

        if not json_file or not json_schema:
            log.warning("No Params and Vars file specified for {}".format(self.associated_class.node_name))
            return parameters, variables

        # absolute location of the JSON file
        # TODO - assumption: AADL and XML file locations are the same
        json_file = os.path.join(global_filepath.aadl_model_dir, json_file)
        json_schema = os.path.join(global_filepath.aadl_model_dir, json_schema)
        json_base_schema = json_file = sys.path[0]+"/internal_state_base.schema.json"

        try:
            loaded_json_file = json.load(open(json_file))
            loaded_json_schema = json.load(open(json_schema))
            loaded_json_base_schema = json.load(open(json_base_schema))
        except json.JSONDecodeError:
            print("invalid json file")
            return parameters, variables

        print(json.dumps(loaded_json_file))
        print(json.dumps(loaded_json_schema))

        try:
            jsonschema.validate(loaded_json_file, loaded_json_base_schema)
            jsonschema.validate(loaded_json_file, loaded_json_schema)
        except jsonschema.exceptions.ValidationError:
            print("validation failed")
            return parameters, variables

        return parameters, variables

    # Ogni process/node ha un file che descrive params e vars, questo metodo si occupa
    # di processarlo ed aggiungere la variabili e parametri corretti
    def getStateASN(self):
        parameters = []
        variables = []

        # @TODO: per ora è assegnato al process come Source_Text, vedere se è da cambiare
        asn_file = tfs.getSourceText(self.process)

        if asn_file is None:
            log.warning("No Params and Vars ASN.1 file specified for {}".format(self.associated_class.node_name))
            return parameters, variables

        # @TODO: il percorso del file AADL è lo stesso del file XML
        ocarina_ros_path = global_filepath.aadl_model_dir
        parsed_asn = parse_file(os.path.join(ocarina_ros_path, asn_file))

        # Estraggo tutti i tipi che NON sono Params o Vars
        custom_types = []
        for t in parsed_asn[self.STATE_ASN_DEFINITION]['types']:
            if t != self.STATE_ASN_PARAMS and \
                    t != self.STATE_ASN_VARS:
                custom_types.append(t)

        params_asn = parsed_asn[self.STATE_ASN_DEFINITION]['types'][self.STATE_ASN_PARAMS]['members']
        vars_asn = parsed_asn[self.STATE_ASN_DEFINITION]['types'][self.STATE_ASN_VARS]['members']

        for index, p_v in enumerate(params_asn + vars_asn):

            tmp_param = self.getVariableFromASN1(p_v)

            if index < len(params_asn):
                parameters.append(tmp_param)
            else:
                variables.append(tmp_param)

        # TODO remove this
        print(parameters)
        print(variables)
        return parameters, variables

    def createHandlerForParameter(self, p, struct_name=""):

        if isinstance(p, Struct):
            new_struct_name = "{}/".format(p.create_instance_with_name)
            if len(struct_name) > 0:
                new_struct_name = "{}/{}".format(struct_name, new_struct_name)

            for p_s in p.variables:
                self.createHandlerForParameter(p_s, new_struct_name)

        else:
            if p.hasDefaultValue():
                self.prepare.addMiddleCode("handle.param<{}>(\"{}{}\", p.{}{}, {});"
                                           .format(p.type.generateCode(),
                                                   struct_name,
                                                   p.name,
                                                   struct_name.replace("/", "."),  # Al posto dello / ha il .
                                                   p.name,
                                                   p.default_value))
            else:
                self.prepare.addMiddleCode("handle.getParam(\"{}\", p.{});"
                                           .format(p.name, p.name))

            log.info("Parameters: {}".format(p.generateCode()))

    def populateData(self):
        # Ottengo tutti i dati relativi al main thread
        (prepare, tearDown, errorHandler) = tfs.getMainThreadFunctions(self.thread)

        self.prepare = Prepare(self.associated_class)
        self.prepare.source_text_function = self.createSourceTextFileFromSourceText(tfs.getSourceText(prepare),
                                                                                    "custom_prepare")

        self.tearDown = TearDown(self.associated_class)
        self.tearDown.source_text_function = self.createSourceTextFileFromSourceText(tfs.getSourceText(tearDown),
                                                                                     "custom_teardown")

        # Aggiungo la chiamata alla funzione custome
        if self.tearDown.source_text_function:
            code = "{};".format(self.tearDown.source_text_function.generateInlineCode())
            self.tearDown.addMiddleCode(code)

        self.errorHandling = ErrorHandling(self.associated_class)
        self.errorHandling.source_text_function = self.createSourceTextFileFromSourceText(
            tfs.getSourceText(errorHandler),
            "custom_errorHandling")

        # Aggiungo la chiamata alla funzione custome
        if self.errorHandling.source_text_function:
            code = "{};".format(self.errorHandling.source_text_function.generateInlineCode())
            self.errorHandling.addBottomCode(code)

        self.associated_class.addPrivateMethod(self.prepare)
        self.associated_class.addPrivateMethod(self.tearDown)
        self.associated_class.addPrivateMethod(self.errorHandling)

        # Ottengo lo stato (parameters, variables) in ASN.1 del nodo
        # (parameters, variables) = self.getStateASN()
        (parameters, variables) = self.getStateJSON()

        # Se ho almeno parametri o variabili, allora genero
        # anche la node configuration, altrimenti no
        if len(parameters) > 0 or len(variables) > 0:
            type_internal_state = Type()
            type_internal_state.setTypeName("InternalState")

            internal_state_is = Variable(self.associated_class)
            internal_state_is.setName("is")
            internal_state_is.setType(type_internal_state)
            self.associated_class.addInternalVariable(internal_state_is)

            node_conf = NodeConfiguration(self.associated_class)
            self.associated_class.setNodeConfiguration(node_conf)

            if len(variables) > 0:
                vars_struct = VariablesStruct(self.associated_class)

                for v in variables:
                    vars_struct.addVariable(v)

                self.associated_class.node_configuration.addStruct(vars_struct)
                self.associated_class.node_configuration.has_variables = True

            # Se ho dei parametri, allora genero la struct
            if len(parameters) > 0:
                params_struct = ParametersStruct(self.associated_class)
                self.prepare.addTopCode("Parameters p;")

                for p in parameters:
                    params_struct.addVariable(p)
                    self.createHandlerForParameter(p)

                self.prepare.addMiddleCode("is.initialize(&p);")
                self.associated_class.node_configuration.addStruct(params_struct)
                self.associated_class.node_configuration.has_parameters = True
            else:
                self.prepare.addMiddleCode("is.initialize();")

        # Aggiungo la chiamata alla funzione custome
        if self.prepare.source_text_function:
            self.prepare.source_text_function.setFunctionType(Bool(self.associated_class))
            code = "return {};".format(self.prepare.source_text_function.generateInlineCode())
            self.prepare.addBottomCode(code)
        else:
            # Return alla fine del metodo main
            self.prepare.addBottomCode("return true;")

        return True, ""
