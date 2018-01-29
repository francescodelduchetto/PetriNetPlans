from AbstractTopicCondition import AbstractTopicCondition
from tf2_msgs.msg import TFMessage

class TfStatic(AbstractTopicCondition):

    _topic_name = "/tf_static"

    _topic_type = TFMessage

    def _get_value_from_data(self, data):
        tra = data.transforms.transform.translation
        rot = data.transforms.transform.rotation
        return (data.transforms.header.frame_id, data.transforms.child_frame_id, tra.x, tra.y, tra.z, rot.x, rot.y, rot.z, rot.w)

    def evaluate(self, params):
        node = str(params[0])

        return node == self.last_value
