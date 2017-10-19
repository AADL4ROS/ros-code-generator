from CObject import CObject
from comments.Comment import Comment
import hashlib

class Method(CObject):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.namespace          = None
        self.method_name        = None
        self.return_type        = None
        self.input_params       = []
        self.library            = None
        self.codice_at_top      = []
        self.codice_at_middle   = []
        self.codice_at_bottom   = []
        self.source_text_file   = None

    def setLibrary(self, _library):
        self.library = _library
        self.associated_class.addLibrary( _library )

    def removeLibrary(self):
        self.associated_class.removeLibrary( self.library )
        self.library = None

    def addInputParameter(self, _param):
        self.input_params.append( _param )

    def shouldImportLibrary(self):
        return (self.library != None)

    def addTopCode(self, _codes):
        if isinstance(_codes, str):
            self.codice_at_top.extend( _codes.split("\n") )
        elif isinstance(_codes, CObject):
            self.codice_at_top.append( _codes )

    def addMiddleCode(self, _codes):
        if isinstance(_codes, str):
            self.codice_at_middle.extend( _codes.split("\n") )
        elif isinstance(_codes, CObject):
            self.codice_at_middle.append( _codes )

    def addBottomCode(self, _codes):
        if isinstance(_codes, str):
            self.codice_at_bottom.extend( _codes.split("\n") )
        elif isinstance(_codes, CObject):
            self.codice_at_bottom.append( _codes )

    def getThreadPointer(self):
        namespace = ""

        if self.namespace != None:
            namespace = "{}::".format(self.namespace)

        return "&{}{}".format(namespace, self.method_name)

    def getIntestazione(self, with_namespace = True):
        namespace = ""

        if with_namespace:
            if self.namespace != None:
                namespace = "{}::".format(self.namespace)

        code = "{} {}{}(".format(self.return_type.generateCode(), namespace, self.method_name)

        params = ", ".join(p.generateCode() for p in self.input_params)
        code += params

        code += ")"
        return code

    def generateInterface(self):
        return "{};".format( self.getIntestazione(with_namespace = False) )

    def isEqualTo(self, another_method, compare_namespace = False):
        # NON faccio sempre il controllo anche del namespace, perchè
        # se devo vedere il nome ed il contenuto in nodi diversi, questi
        # avranno namespace diversi
        if compare_namespace and self.namespace != another_method.namespace:
            return False

        if self.method_name != another_method.method_name:
            return False

        if not self.return_type.isEqualTo(another_method.return_type):
            return False

        if self.library != None:
            if not self.library.isEqualTo(another_method.library):
                return False
        else:
            if another_method.library != None:
                return False

        if self.source_text_file != None:
            if not self.source_text_file.isEqualTo(another_method.source_text_file):
                return False
        else:
            if another_method.source_text_file != None:
                return False

        # Per controllare il contenuto del metodo, genero tutto il codice e controllo
        # che sia identico per i due metodi. Se necessario rimuovo i namespace che sono
        # ovviamente diversi per nodi con nomi diversi, anche se i due nodi sono identici
        # in tutto il resto.
        code            = self.generateCode()
        another_code    = another_method.generateCode()

        # Tolgo i riferimenti alla classe
        code            = code.replace(self.associated_class.class_name, "")
        another_code    = another_code.replace(another_method.associated_class.class_name, "")

        if not compare_namespace:
            if self.namespace != None:
                code = code.replace(self.namespace, "")
            if another_method.namespace != None:
                another_code = another_code.replace(another_method.namespace, "")

        # Comparo il digest MD5 dei due codici: in questo modo dovrei ovviare a possibili
        # rallentamenti con codici estremamenti lunghi. Apre però la possibilità a collisioni
        # fra i due hash anche se le stringhe sono diverse. E' comunque una casistica abbastanza
        # rara da poter essere trascurata nel nostro caso.
        code_md5            = hashlib.md5()
        another_code_md5    = hashlib.md5()

        code_md5.update(bytes(code, encoding="utf-8"))
        another_code_md5.update(bytes(another_code, encoding="utf-8"))

        if code_md5.digest() != another_code_md5.digest():
            return False

        return True

    def generateCode(self):
        comment = Comment()
        comment.setComment( "Method {} auto-generated".format(self.method_name) )
        code = comment.generateCode()

        code += "{} {{\n".format( self.getIntestazione(with_namespace = True) )

        for s in self.codice_at_top + self.codice_at_middle + self.codice_at_bottom:
            if isinstance(s, str):
                if len(s) > 0:
                    code += "\t" + s + "\n"
            elif isinstance(s, CObject):
                code += "".join(["\t" + gen_s + "\n" for gen_s in s.generateCode().split("\n")])

        # Chiamata a funzione custom DOPO tutto il resto del codice

        code += "}\n"
        code += "\n"

        return code