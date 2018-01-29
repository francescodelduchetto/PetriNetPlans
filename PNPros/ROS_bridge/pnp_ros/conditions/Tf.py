from AbstractTopicCondition import AbstractTopicCondition
from tf2_msgs.msg import TFMessage

class Tf(AbstractTopicCondition):

    _topic_name = "/tf"

    _topic_type = TFMessage

    def _get_value_from_data(self, data):
        # TODO this just consider the first...
        tra = data.transforms[0].transform.translation
        rot = data.transforms[0].transform.rotation
        return (data.transforms[0].header.frame_id, data.transforms[0].child_frame_id, tra.x, tra.y, tra.z, rot.x, rot.y, rot.z, rot.w)

    def evaluate(self, params):
        node = str(params[0])

        return node == self.last_value
