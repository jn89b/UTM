class AirsimROSWrapper():
    """
    takes in information of uav
    instantiates airsimdroneros for n uavs and appends to list
    then publishes the wrapped messages
    """

    def __init__(self, wsl_ip):
        """Houses the API calls"""
        self.client = airsim.MultirotorClient(ip=str(wsl_ip))

    def __get_global_uav_location(self, uav_name):
        """get orientation of UAV"""
        return self.client.simGetObjectPose(uav_name) 

    def __get_local_uavs(uav_name):
        """get local uav locations"""
        #print("Local Position of UAV"+str(i), client.simGetVehiclePose("PX4_"+str(i)))
        return self.client.simGetVehiclePose(uav_name)

    def __convert_ned_to_enu(self, ned_pose):
        """convert ned convention of airsim to ned for ROS publishing"""
        enu_x = ned_pose.position.y_val
        enu_y = ned_pose.position.x_val
        enu_z = -ned_pose.position.z_val
        enu_quat_x =  ned_pose.orientation.y_val
        enu_quat_y =  ned_pose.orientation.x_val
        enu_quat_z =  -ned_pose.orientation.x_val
        enu_quat_w =  ned_pose.orientation.w_val

        enu_position = [enu_x, enu_y, enu_z]
        enu_quat = [enu_quat_x, enu_quat_y, enu_quat_z, enu_quat_w]

        return enu_position, self.__normalize_quat(enu_quat)

    def __normalize_quat(self, quat_list):
        """normalize the quaternion"""
        quat_vector = np.array(quat_list)
        magnitude =  np.linalg.norm(quat_vector)
        
        return quat_vector/magnitude
