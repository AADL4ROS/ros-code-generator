from libraries.Library import Library

kGEOMETRY_MSGS_NAMESPACE = "geometry_msgs"


class Accel(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/Accel.h".format(kGEOMETRY_MSGS_NAMESPACE)

class AccelStamped(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/AccelStamped.h".format(kGEOMETRY_MSGS_NAMESPACE)

class AccelWithCovariance(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/AccelWithCovariance.h".format(kGEOMETRY_MSGS_NAMESPACE)

class AccelWithCovarianceStamped(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/AccelWithCovarianceStamped.h".format(kGEOMETRY_MSGS_NAMESPACE)

class Inertia(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/Inertia.h".format(kGEOMETRY_MSGS_NAMESPACE)

class InertiaStamped(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/InertiaStamped.h".format(kGEOMETRY_MSGS_NAMESPACE)

class Point(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/Point.h".format(kGEOMETRY_MSGS_NAMESPACE)

class Point32(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/Point32.h".format(kGEOMETRY_MSGS_NAMESPACE)

class PointStamped(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/PointStamped.h".format(kGEOMETRY_MSGS_NAMESPACE)

class Polygon(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/Polygon.h".format(kGEOMETRY_MSGS_NAMESPACE)

class PolygonStamped(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/PolygonStamped.h".format(kGEOMETRY_MSGS_NAMESPACE)

class Pose(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/Pose.h".format(kGEOMETRY_MSGS_NAMESPACE)

class Pose2D(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/Pose2D.h".format(kGEOMETRY_MSGS_NAMESPACE)

class PoseArray(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/PoseArray.h".format(kGEOMETRY_MSGS_NAMESPACE)

class PoseStamped(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/PoseStamped.h".format(kGEOMETRY_MSGS_NAMESPACE)

class PoseWithCovariance(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/PoseWithCovariance.h".format(kGEOMETRY_MSGS_NAMESPACE)

class PoseWithCovarianceStamped(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/PoseWithCovarianceStamped.h".format(kGEOMETRY_MSGS_NAMESPACE)

class Quaternion(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/Quaternion.h".format(kGEOMETRY_MSGS_NAMESPACE)

class QuaternionStamped(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/QuaternionStamped.h".format(kGEOMETRY_MSGS_NAMESPACE)

class Transform(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/Transform.h".format(kGEOMETRY_MSGS_NAMESPACE)

class TransformStamped(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/TransformStamped.h".format(kGEOMETRY_MSGS_NAMESPACE)

class Twist(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/Twist.h".format(kGEOMETRY_MSGS_NAMESPACE)

class TwistStamped(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/TwistStamped.h".format(kGEOMETRY_MSGS_NAMESPACE)

class TwistWithCovariance(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/TwistWithCovariance.h".format(kGEOMETRY_MSGS_NAMESPACE)

class TwistWithCovarianceStamped(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/TwistWithCovarianceStamped.h".format(kGEOMETRY_MSGS_NAMESPACE)

class Vector3(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/Vector3.h".format(kGEOMETRY_MSGS_NAMESPACE)

class Vector3Stamped(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/Vector3Stamped.h".format(kGEOMETRY_MSGS_NAMESPACE)

class Wrench(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/Wrench.h".format(kGEOMETRY_MSGS_NAMESPACE)

class WrenchStamped(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "{}/WrenchStamped.h".format(kGEOMETRY_MSGS_NAMESPACE)