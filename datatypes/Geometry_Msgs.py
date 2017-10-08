from datatypes.Type import Type
import libraries.Geometry_Msgs as geometry_msgs


class Accel(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "Accel" )
        self.setLibrary( geometry_msgs.Accel() )

class AccelStamped(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "AccelStamped" )
        self.setLibrary( geometry_msgs.AccelStamped() )

class AccelWithCovariance(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "AccelWithCovariance" )
        self.setLibrary( geometry_msgs.AccelWithCovariance() )

class AccelWithCovarianceStamped(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "AccelWithCovarianceStamped" )
        self.setLibrary( geometry_msgs.AccelWithCovarianceStamped() )

class Inertia(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "Inertia" )
        self.setLibrary( geometry_msgs.Inertia() )

class InertiaStamped(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "InertiaStamped" )
        self.setLibrary( geometry_msgs.InertiaStamped() )

class Point(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "Point" )
        self.setLibrary( geometry_msgs.Point() )

class Point32(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "Point32" )
        self.setLibrary( geometry_msgs.Point32() )

class PointStamped(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "PointStamped" )
        self.setLibrary( geometry_msgs.PointStamped() )

class Polygon(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "Polygon" )
        self.setLibrary( geometry_msgs.Polygon() )

class PolygonStamped(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "PolygonStamped" )
        self.setLibrary( geometry_msgs.PolygonStamped() )

class Pose(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "Pose" )
        self.setLibrary( geometry_msgs.Pose() )

class Pose2D(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "Pose2D" )
        self.setLibrary( geometry_msgs.Pose2D() )

class PoseArray(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "PoseArray" )
        self.setLibrary( geometry_msgs.PoseArray() )

class PoseStamped(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "PoseStamped" )
        self.setLibrary( geometry_msgs.PoseStamped() )

class PoseWithCovariance(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "PoseWithCovariance" )
        self.setLibrary( geometry_msgs.PoseWithCovariance() )

class PoseWithCovarianceStamped(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "PoseWithCovarianceStamped" )
        self.setLibrary( geometry_msgs.PoseWithCovarianceStamped() )

class Quaternion(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "Quaternion" )
        self.setLibrary( geometry_msgs.Quaternion() )

class QuaternionStamped(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "QuaternionStamped" )
        self.setLibrary( geometry_msgs.QuaternionStamped() )

class Transform(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "Transform" )
        self.setLibrary( geometry_msgs.Transform() )

class TransformStamped(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "TransformStamped" )
        self.setLibrary( geometry_msgs.TransformStamped() )

class Twist(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "Twist" )
        self.setLibrary( geometry_msgs.Twist() )

class TwistStamped(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "TwistStamped" )
        self.setLibrary( geometry_msgs.TwistStamped() )

class TwistWithCovariance(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "TwistWithCovariance" )
        self.setLibrary( geometry_msgs.TwistWithCovariance() )

class TwistWithCovarianceStamped(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "TwistWithCovarianceStamped" )
        self.setLibrary( geometry_msgs.TwistWithCovarianceStamped() )

class Vector3(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "Vector3" )
        self.setLibrary( geometry_msgs.Vector3() )

class Vector3Stamped(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "Vector3Stamped" )
        self.setLibrary( geometry_msgs.Vector3Stamped() )

class Wrench(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "Wrench" )
        self.setLibrary( geometry_msgs.Wrench() )

class WrenchStamped(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std_msgs" )
        self.setTypeName( "WrenchStamped" )
        self.setLibrary( geometry_msgs.WrenchStamped() )