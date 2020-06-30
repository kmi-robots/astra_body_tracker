#include "ros/ros.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <astra_body_tracker/BodyListStamped.h>

#include <astra/astra.hpp>
#include <cstdio>
#include <iostream>

class BodyFrameListener : public astra::FrameListener {
public:
    BodyFrameListener() = default;

    explicit BodyFrameListener(const ros::Publisher &p) {
        bodyListPub_ = p;
    }

    bool is_finished() const { return isFinished_; }

private:
    void on_frame_ready(astra::StreamReader &reader, astra::Frame &frame) override {
        processBodies(frame);
    }

    void processBodies(astra::Frame &frame) {
        astra::BodyFrame bodyFrame = frame.get<astra::BodyFrame>();
//        const float jointScale = bodyFrame.info().width() / 120.f;
        const auto &bodies = bodyFrame.bodies();

        astra_body_tracker::BodyListStamped bodyList;
        ros::Time sampleTime = ros::Time::now();

        bodyList.header.stamp = sampleTime;
        bodyList.header.frame_id = "world";

        for (auto &body : bodies) {
            if (body.status() == astra::BodyStatus::Tracking) {
                ROS_INFO_STREAM("Processing frame #"<<bodyFrame.frame_index()<<" body "<<std::to_string(body.id()));
                bodyList.ids.push_back(body.id());
                std::string id = std::to_string(body.id());
                for (auto &joint : body.joints()) {
                    if(joint.status() == astra::JointStatus::Tracked) {
                        switch (joint.type()) {
                            case astra::JointType::Head:
                                publishJoint("head_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::ShoulderSpine:
                                publishJoint("shoulder_spine_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::LeftShoulder:
                                publishJoint("left_shoulder_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::LeftElbow:
                                publishJoint("left_elbow_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::LeftHand:
                                publishJoint("left_hand_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::RightShoulder:
                                publishJoint("right_shoulder_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::RightElbow:
                                publishJoint("right_elbow_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::RightHand:
                                publishJoint("right_hand_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::MidSpine:
                                publishJoint("torso_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::BaseSpine:
                                publishJoint("base_spine_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::LeftHip:
                                publishJoint("left_hip_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::LeftKnee:
                                publishJoint("left_knee_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::LeftFoot:
                                publishJoint("left_foot_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::RightHip:
                                publishJoint("right_hip_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::RightKnee:
                                publishJoint("right_knee_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::RightFoot:
                                publishJoint("right_foot_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::LeftWrist:
                                publishJoint("left_wrist_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::RightWrist:
                                publishJoint("right_wrist_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::Neck:
                                publishJoint("neck_" + id, sampleTime, joint.world_position(), joint.orientation());
                                break;
                            case astra::JointType::Unknown:
                                break;
                        }
                    }
                }
            }
        }

        if (!bodyList.ids.empty()) {
            bodyListPub_.publish(bodyList);
        }


        const auto &floor = bodyFrame.floor_info(); //floor
        if (floor.floor_detected()) {
            const auto &p = floor.floor_plane();
            ROS_INFO_STREAM("Floor plane: ["<<p.a()<<", "<<p.b()<<", "<<p.c()<<", "<<p.d()<<"]");

        }
    }

    void publishJoint(const std::string &frame_id, ros::Time sampleTime, const astra::Vector3f &coordinates, const astra::Matrix3x3 &orientation) {
        if(coordinates.x + coordinates.y + coordinates.z == 0)
            return;
        ROS_INFO_STREAM(frame_id<<" is in position "<<coordinates.x<<", "<<coordinates.y<<", "<<coordinates.z);
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = sampleTime;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = frame_id;
        transformStamped.transform.translation.x = coordinates.x / 1000;
        transformStamped.transform.translation.y = coordinates.y / 1000;
        transformStamped.transform.translation.z = coordinates.z / 1000;
        tf2::Matrix3x3 m;
        m.setValue(orientation.m00(), orientation.m01(), orientation.m02(),
                   orientation.m10(), orientation.m11(), orientation.m12(),
                   orientation.m20(), orientation.m21(), orientation.m22());
        tf2::Quaternion q;
        m.getRotation(q);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br_.sendTransform(transformStamped);
    }

    bool isFinished_{false};
    tf2_ros::TransformBroadcaster br_;
    ros::Publisher bodyListPub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "astra_cpp_body_tracker");
    ros::NodeHandle node;
    astra::initialize();
    astra::StreamSet streamSet;
    astra::StreamReader reader = streamSet.create_reader();
    const char *licenseString = "";
    orbbec_body_tracking_set_license(licenseString);

    BodyFrameListener listener(node.advertise<astra_body_tracker::BodyListStamped>("body_list", 1));

    auto bodyStream = reader.stream<astra::BodyStream>();
    bodyStream.start();
    reader.add_listener(listener);

//    astra::SkeletonProfile profile = bodyStream.get_skeleton_profile();

    // HandPoses includes Joints and Segmentation
//    astra::BodyTrackingFeatureFlags features = astra::BodyTrackingFeatureFlags::HandPoses;

    do {
        astra_update();
        ros::spinOnce();
    } while (!listener.is_finished());

    reader.remove_listener(listener);
    astra::terminate();
    return 0;
}
