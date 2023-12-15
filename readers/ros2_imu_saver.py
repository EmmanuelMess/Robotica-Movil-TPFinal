from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


def main():
    with Reader('agenteTopics') as reader:
        with open("imu.csv", "w") as file:
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == '/imu':
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    time = int(msg.header.stamp.sec*1e9) + int(msg.header.stamp.nanosec)
                    
                    file.write(f"{time},"
                               f"{msg.angular_velocity.x},{msg.angular_velocity.y},{msg.angular_velocity.z},"
                               f"{msg.linear_acceleration.x},{msg.linear_acceleration.y},{msg.linear_acceleration.z}\n")
                
if __name__ == '__main__':
    main()
