#! /usr/bin/env python

import rospy
from node_imu_reader import cImuReader
from std_srvs.srv import Trigger, TriggerResponse
import numpy as np


class cCollisionDetectionService(cImuReader):
    def __init__(self, _srv_name='/crash_direction_service'):
        cImuReader.__init__(self, _rate=0.01)
        self.collition_info_ = None
        self._my_service_ = rospy.Service(_srv_name, Trigger , self.srv_callback)
        self._threshhold = 7.00

    def in_spin(self):
        imu_data = self.get_data()
        acc = np.array([getattr(imu_data.linear_acceleration, comp) for comp in 'xyz'])

        rospy.loginfo('blabla')
        rospy.loginfo(str(type(acc)))
        if np.linalg.norm(acc) > self._threshhold:
            self.collition_info_ = acc
        rospy.loginfo('blabla-------------')
        

    def srv_callback(self, request):
        
        response = TriggerResponse()
        if self.collition_info_ is None:
            response.success = False
            response.message = 'forwards'
            return response

        response.success=True

        acc = service.collition_info_ 
        service.collition_info_ = None
        index = np.argmax(np.abs(acc))

        val = acc[index]
        aux_array = [('forwards', 'backwards'), ('left', 'right'), ('up', 'down')]

        if vel > 0:
            response.message = aux_array[index][0]
        else:
            response.message = aux_array[index][1]


        return response

if __name__ == '__main__':
    rospy.init_node('collision_detector')
    service = cCollisionDetectionService()
    service.spin()
