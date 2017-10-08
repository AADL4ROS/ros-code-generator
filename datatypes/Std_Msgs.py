from datatypes.Type import Type
import libraries.Std_Msgs as std_msgs


class Bool(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Bool")
        self.setLibrary(std_msgs.Bool())


class Byte(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Byte")
        self.setLibrary(std_msgs.Byte())


class ByteMultiArray(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("ByteMultiArray")
        self.setLibrary(std_msgs.ByteMultiArray())


class Char(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Char")
        self.setLibrary(std_msgs.Char())


class ColorRGBA(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("ColorRGBA")
        self.setLibrary(std_msgs.ColorRGBA())


class Duration(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Duration")
        self.setLibrary(std_msgs.Duration())


class Empty(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Empty")
        self.setLibrary(std_msgs.Empty())


class Float32(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Float32")
        self.setLibrary(std_msgs.Float32())


class Float32MultiArray(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Float32MultiArray")
        self.setLibrary(std_msgs.Float32MultiArray())


class Float64(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Float64")
        self.setLibrary(std_msgs.Float64())


class Float64MultiArray(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Float64MultiArray")
        self.setLibrary(std_msgs.Float64MultiArray())


class Header(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Header")
        self.setLibrary(std_msgs.Header())


class Int16(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Int16")
        self.setLibrary(std_msgs.Int16())


class Int16MultiArray(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Int16MultiArray")
        self.setLibrary(std_msgs.Int16MultiArray())


class Int32(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Int32")
        self.setLibrary(std_msgs.Int32())


class Int32MultiArray(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Int32MultiArray")
        self.setLibrary(std_msgs.Int32MultiArray())


class Int64(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Int64")
        self.setLibrary(std_msgs.Int64())


class Int64MultiArray(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Int64MultiArray")
        self.setLibrary(std_msgs.Int64MultiArray())


class Int8(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Int8")
        self.setLibrary(std_msgs.Int8())


class Int8MultiArray(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Int8MultiArray")
        self.setLibrary(std_msgs.Int8MultiArray())


class MultiArrayDimension(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("MultiArrayDimension")
        self.setLibrary(std_msgs.MultiArrayDimension())


class MultiArrayLayout(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("MultiArrayLayout")
        self.setLibrary(std_msgs.MultiArrayLayout())


class String(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("String")
        self.setLibrary(std_msgs.String())


class Time(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("Time")
        self.setLibrary(std_msgs.Time())


class UInt16(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("UInt16")
        self.setLibrary(std_msgs.UInt16())


class UInt16MultiArray(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("UInt16MultiArray")
        self.setLibrary(std_msgs.UInt16MultiArray())


class UInt32(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("UInt32")
        self.setLibrary(std_msgs.UInt32())


class UInt32MultiArray(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("UInt32MultiArray")
        self.setLibrary(std_msgs.UInt32MultiArray())


class UInt64(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("UInt64")
        self.setLibrary(std_msgs.UInt64())


class UInt64MultiArray(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("UInt64MultiArray")
        self.setLibrary(std_msgs.UInt64MultiArray())


class UInt8(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("UInt8")
        self.setLibrary(std_msgs.UInt8())


class UInt8MultiArray(Type):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setNamespace("std_msgs")
        self.setTypeName("UInt8MultiArray")
        self.setLibrary(std_msgs.UInt8MultiArray())
