import rospy
from nav_msgs.msg import OccupancyGrid
import tf
import copy

class WifiMap:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
        self.sub = rospy.Subscriber("map", OccupancyGrid, self._callback_map)
        self.map = None
        self.map_copied = None

    def _callback_map(self, msg_map):
        self.map = msg_map
        self.map_copied = copy.deepcopy(msg_map)

    def get_global_position(self):
        if self.map is None:
            return None

        now = rospy.Time.now()
        self.listener.waitForTransform("/map", "/base_link", now, rospy.Duration(3.0))
        trans, _ = self.listener.lookupTransform("/map", "/base_link", rospy.Time((0)))
        pos_map_origin = self.map.info.origin.position
        x = - pos_map_origin.x + trans[0]
        y = - pos_map_origin.y + trans[1]
        return x, y

    def create_debug_map(self):
        ret = self.get_global_position()
        if ret is None:
            return None
        x, y = ret

        if self.map_copied is None:
            return None

        info = self.map_copied.info
        resol = info.resolution
        data = [0 for i in range(len(self.map_copied.data))]
        idx_x = int(x // resol)
        idx_y = int(y // resol)
        to_idx = lambda i, j: j * info.height + i
        for i in [idx_x - 50 + i for i in range(101)]:
            for j in [idx_y - 50 + j for j in range(101)]:
                idx = to_idx(i, j)
                data[idx] = 1.0
        self.map_copied.data = data
        return self.map_copied


