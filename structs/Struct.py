from CObject import CObject


class Struct(CObject):
    def __init__(self, _associated_class, name):
        super().__init__( _associated_class )

        # Il nome della struct
        self.name = name

        # Le variabili interne alla struct
        self.variables = []

        # Se la struct ha una superclass da cui eredita
        self.super_class = None

        # Se la struct ha un costruttore o meno. Se lo ha allora
        # a tutte le variabili è assegnato il loro valore di defualt
        self.has_constructor = False

    def addVariable(self, var):
        self.variables.append(var)

    def generateCode(self):
        code = ""

        # Intestazione della struct
        code += "struct {}".format(self.name)

        if self.super_class != None:
            code += ": {}".format(self.super_class)

        code += "{ \n"

        # Con l'argomento generate_default_value = False dico di non generare la default value
        # anche se questa è presente
        for v in self.variables:
            code += "\t{}\n".format( v.generateCode(generate_default_value = False) )

        # Se ha il costruttore, allora lo genero
        if self.has_constructor:
            code += "\t{} () {{\n".format(self.name)

            for v in self.variables:
                code += "\t\t{}\n".format( v.generateCodeOnlyAssignment() )

            code += "\t};\n"

        # Chiusura della struct
        code += "};"

        return code