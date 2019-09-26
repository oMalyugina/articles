//
// Created by olga on 23.09.19.
//


#include <KittyToRosConverter.h>

void KittiConverter::convertImu() {
    string folderWithImu = this->pathToFolder + "/oxts/data";
    string fileWithTimestamps = this->pathToFolder + "/oxts/timestamps.txt";

    std::vector<uint64_t> timestamp = loadTimestampsIntoVector(fileWithTimestamps);
    for (int frame = 0; frame < timestamp.size(); ++frame) {
        string fileWithImu = folderWithImu + "/" + getFilenameForEntry(frame) + ".txt";
        std::ifstream imuData(fileWithImu, std::ios::in);
        std::string line;
        std::vector<double> parsed_doubles;
        std::getline(imuData, line);
        PoseOxts pose = PoseOxts(line);
        pose.imu_msg.header.stamp = ros::Time(timestamp[frame] / 1000000000, timestamp[frame] % 1000000000);
        bag.write("imu", pose.imu_msg.header.stamp, pose.imu_msg);

    }
}

std::string KittiConverter::getNameOfFile(uint64_t frame) const {
    char buffer[20];
    sprintf(buffer, "%010llu", frame);
    return std::string(buffer);
}

std::vector<uint64_t> KittiConverter::loadTimestampsIntoVector(const std::string &filename) const {
    std::vector<uint64_t> timestamp_vec;
    std::ifstream import_file(filename, std::ios::in);
    if (!import_file) {
        std::cerr << "bad timestamp for imu" << std::endl;
        return timestamp_vec;
    }

    std::string line;
    while (std::getline(import_file, line)) {
        std::stringstream line_stream(line);

        std::string timestamp_string = line_stream.str();
        std::tm t = {};
        t.tm_year = std::stoi(timestamp_string.substr(0, 4)) - 1900;
        t.tm_mon = std::stoi(timestamp_string.substr(5, 2)) - 1;
        t.tm_mday = std::stoi(timestamp_string.substr(8, 2));
        t.tm_hour = std::stoi(timestamp_string.substr(11, 2));
        t.tm_min = std::stoi(timestamp_string.substr(14, 2));
        t.tm_sec = std::stoi(timestamp_string.substr(17, 2));
        t.tm_isdst = -1;

        static const uint64_t kSecondsToNanoSeconds = 1e9;
        time_t time_since_epoch = mktime(&t);

        uint64_t timestamp = time_since_epoch * kSecondsToNanoSeconds +
                             std::stoi(timestamp_string.substr(20, 9));
        timestamp_vec.push_back(timestamp);
    }

    return timestamp_vec;
}

void KittiConverter::convertPointCloud() {
    string folderWithPoints = this->pathToFolder + "/velodyne_points/data/";
    string fileWithTimestamps = this->pathToFolder + "/velodyne_points/timestamps.txt";

    std::vector<uint64_t> timestamp = loadTimestampsIntoVector(fileWithTimestamps);

    for (int frame = 0; frame < timestamp.size(); ++frame) {
        std::string filename = folderWithPoints + getNameOfFile(frame) + ".bin";

        std::ifstream input(filename, std::ios::in | std::ios::binary);
        if (!input) {
            std::cout << "Could not open pointcloud file.\n";
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
        for (size_t i = 0; input.good() && !input.eof(); i++) {
            pcl::PointXYZI point;
            input.read((char *) &point.x, 3 * sizeof(float));
            input.read((char *) &point.intensity, sizeof(float));
            cloudPtr->push_back(point);
        }
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloudPtr, msg);
        msg.header.stamp = ros::Time(timestamp[frame] / 1000000000, timestamp[frame] % 1000000000);
        msg.header.frame_id = "horizontal_vlp16_link";
        bag.write("horizontal_laser_3d", ros::Time(timestamp[frame] / 1000000000, timestamp[frame] % 1000000000), msg);
    }
}

void KittiConverter::convertImuAndGps() {
    string folderWithImu = this->pathToFolder + "/oxts/data";
    string fileWithTimestamps = this->pathToFolder + "/oxts/timestamps.txt";

    std::vector<uint64_t> timestamp = loadTimestampsIntoVector(fileWithTimestamps);
    for (int frame = 0; frame < timestamp.size(); ++frame) {
        string fileWithImu = folderWithImu + "/" + getNameOfFile(frame) + ".txt";
        std::ifstream imuData(fileWithImu, std::ios::in);
        std::string line;
        std::vector<double> parsed_doubles;
        std::getline(imuData, line);
        PoseOxts pose = PoseOxts(line);
        pose.imu_msg.header.stamp = ros::Time(timestamp[frame] / 1000000000, timestamp[frame] % 1000000000);
        bag.write("imu", pose.imu_msg.header.stamp, pose.imu_msg);

        pose.gps_msg.header.stamp = ros::Time(timestamp[frame] / 1000000000, timestamp[frame] % 1000000000);
        bag.write("fix", pose.gps_msg.header.stamp, pose.gps_msg);
    }
}


PoseOxts::PoseOxts(const string &line) {

    std::istringstream oxts(line);
    vector<double> obj_data(30);
    for (int j = 0; j < 30; ++j) {
        oxts >> obj_data[j];
    }

    double lat = obj_data[0];//    latitude of the oxts-unit (deg)
    double lon = obj_data[1];//    longitude of the oxts-unit (deg)
    double alt = obj_data[2];//    altitude of the oxts-unit (m)
    double roll = obj_data[3];//   roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi
    double pitch = obj_data[4];//  pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2
    double yaw = obj_data[5];//    heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi
    double vn = obj_data[6];//     velocity towards north (m/s)
    double ve = obj_data[7];//     velocity towards east (m/s)
    double vf = obj_data[8];//     forward velocity, i.e. parallel to earth-surface (m/s)
    double vl = obj_data[9];//     leftward velocity, i.e. parallel to earth-surface (m/s)
    double vu = obj_data[10];//    upward velocity, i.e. perpendicular to earth-surface (m/s)
    double ax = obj_data[11];//    acceleration in x, i.e. in direction of vehicle front (m/s^2)
    double ay = obj_data[12];//    acceleration in y, i.e. in direction of vehicle left (m/s^2)
    double az = obj_data[13];//    acceleration in z, i.e. in direction of vehicle top (m/s^2)
    double af = obj_data[14];//    forward acceleration (m/s^2)
    double al = obj_data[15];//    leftward acceleration (m/s^2)
    double au = obj_data[16];//    upward acceleration (m/s^2)
    double wx = obj_data[17];//    angular rate around x (rad/s)
    double wy = obj_data[18];//    angular rate around y (rad/s)
    double wz = obj_data[19];//    angular rate around z (rad/s)
    double wf = obj_data[20];//    angular rate around forward axis (rad/s)
    double wl = obj_data[21];//    angular rate around leftward axis (rad/s)
    double wu = obj_data[22];//    angular rate around upward axis (rad/s)
    double pos_accuracy = obj_data[23];//  velocity accuracy (north/east in m)
    double vel_accuracy = obj_data[24];//  velocity accuracy (north/east in m/s)
    double navstat = obj_data[25];//       navigation status (see navstat_to_string)
    double numsats = obj_data[26];//       number of satellites tracked by primary GPS receiver
    double posmode = obj_data[27];//       position mode of primary GPS receiver (see gps_mode_to_string)
    double velmode = obj_data[28];//       velocity mode of primary GPS receiver (see gps_mode_to_string)
    double orimode = obj_data[29];//       orientation mode of primary GPS receiver (see gps_mode_to_string)

    this->imu_msg.header.frame_id = "imu_link";

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    imu_msg.angular_velocity.x = wx;
    imu_msg.angular_velocity.y = wy;
    imu_msg.angular_velocity.z = wz;

    imu_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw (roll, pitch, yaw);

//    this->gps_msg.header.frame_id = "fix_link";
//    this->gps_msg.altitude = alt;
//    this->gps_msg.latitude = lat;
//    this->gps_msg.longitude = lon;
}