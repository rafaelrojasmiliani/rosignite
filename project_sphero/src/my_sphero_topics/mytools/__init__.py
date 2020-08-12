

import rospy


class cMyNode(object):
    def __init__(self):
        self.rate_ = rospy.Rate(1)

    def shutdown(self):
        rospy.loginfo('shutdown time!')
        self.is_running_ = False
        self.move_robot('stop')

    def pre_spin(self):
        pass

    def post_spin(self):
        pass

    def in_spin(self):
        pass

    def spin(self):
        self.pre_spin()

        while self.is_running_:
            self.in_spin()
            self.sleep()

        self.post_spin()

    def sleep(self):
        self.rate_.sleep()

class cTopicReader(cMyNode):
    def __init__(self, _topic_name, _topic_type):

        cMyNode.__init__(self)
        self.susbcriber_ = rospy.Subscriber(_topic_name, _topic_type, self.topic_callback)
        self.topic_name_ = _topic_name
        self.topic_type_ = _topic_type
        self.data_ = None
        rospy.on_shutdown(self.shutdown)
    
    def topic_callback(self, msg):
        self.data_ = msg
    
    def get_data(self):
        return self._data_

    def pre_spin(self)
        rospy.wait_for_message(self.topic_name_, self.topic_type_)



class cTopicWriter(cMyNode):
    def __init__(self, _topic_name, _topic_type):
        cMyNode.__init__(self)
        self.pub_ = rospy.Publisher(_topic_name, _topic_type, queue_size=10)
        self.topic_name_ = _topic_name
        self.topic_type_ = _topic_type
        self.data_ = None

    def build_message(self):
        pass

    def in_spin(self):
        msg = self.get_message()
        self.pub_.publish(msg)
