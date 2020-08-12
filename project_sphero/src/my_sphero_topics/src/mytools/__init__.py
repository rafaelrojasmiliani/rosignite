

import rospy


class cMyNode(object):
    def __init__(self, _rate=1):
        self.rate_ = rospy.Rate(_rate)
        self.is_running_ = True

    def shutdown(self):
        rospy.loginfo('shutdown time!')
        self.is_running_ = False

    def pre_spin(self):
        pass

    def post_spin(self):
        pass

    def in_spin(self):
        pass

    def spin(self):
        self.pre_spin()

        rospy.loginfo('is running' + str(self.is_running_))
        while self.is_running_:
            self.in_spin()
            rospy.loginfo('running' + str(self.is_running_) + str(self.__class__.__name__))
            self.sleep()
        rospy.loginfo('after while')

        self.post_spin()

    def sleep(self):
        self.rate_.sleep()

class cTopicReader(cMyNode):
    def __init__(self, _topic_name, _topic_type, **kwargs):

        cMyNode.__init__(self, **kwargs)
        self.susbcriber_ = rospy.Subscriber(_topic_name, _topic_type, self.topic_callback)
        self.topic_name_ = _topic_name
        self.topic_type_ = _topic_type
        self.data_ = None
        rospy.on_shutdown(self.shutdown)
    
    def topic_callback(self, msg):
        self.data_ = msg
    
    def get_data(self):
        return self.data_

    def pre_spin(self):
        rospy.wait_for_message(self.topic_name_, self.topic_type_)



class cTopicWriter(cMyNode):
    def __init__(self, _topic_name, _topic_type, **kwargs):
        cMyNode.__init__(self, **kwargs)
        self.pub_ = rospy.Publisher(_topic_name, _topic_type, queue_size=10)
        self.topic_name_ = _topic_name
        self.topic_type_ = _topic_type
        self.data_ = None

    def build_message(self):
        pass

    def in_spin(self):
        msg = self.build_message()
        self.pub_.publish(msg)
