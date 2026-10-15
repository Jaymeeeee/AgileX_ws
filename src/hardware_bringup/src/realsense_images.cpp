// Minimal SDK -> ROS publisher: exactly three sensor_msgs/Image topics per camera.
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <librealsense2/rs.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

XmlRpc::XmlRpcValue intrinsics(const rs2::video_stream_profile& profile) {
    const auto i = profile.get_intrinsics();
    XmlRpc::XmlRpcValue out;
    out["width"] = i.width; out["height"] = i.height;
    out["fx"] = double(i.fx); out["fy"] = double(i.fy);
    out["ppx"] = double(i.ppx); out["ppy"] = double(i.ppy);
    out["distortion_model"] = std::string(rs2_distortion_to_string(i.model));
    for (int k=0; k<5; ++k) out["coeffs"][k] = double(i.coeffs[k]);
    return out;
}

bool source_stamp(const rs2::frame& frame, ros::Time& stamp) {
    // Require SDK host/global clock. Never label receipt time as capture time.
    if (frame.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
        return false;
    const double seconds = frame.get_timestamp() * 0.001;
    const double age = ros::Time::now().toSec() - seconds;
    if (!std::isfinite(seconds) || seconds<=0 || age < -0.2 || age > 3.0) return false;
    stamp.fromSec(seconds);
    return true;
}

sensor_msgs::Image image_message(const rs2::video_frame& frame, const ros::Time& stamp,
                                const std::string& frame_id, double depth_scale) {
    sensor_msgs::Image msg;
    msg.header.stamp = stamp; msg.header.frame_id = frame_id;
    msg.width = frame.get_width(); msg.height = frame.get_height();
    const bool depth = frame.get_profile().format() == RS2_FORMAT_Z16;
    if (!depth && frame.get_profile().format() != RS2_FORMAT_RGB8)
        throw std::runtime_error("Unexpected SDK image format");
    msg.encoding = depth ? "16UC1" : "rgb8";
    msg.is_bigendian = false;
    msg.step = msg.width * (depth ? 2 : 3);
    msg.data.resize(msg.step * msg.height);
    const auto* input = static_cast<const uint8_t*>(frame.get_data());
    for (unsigned y=0; y<msg.height; ++y) {
        const auto* row = input + y * frame.get_stride_in_bytes();
        auto* output = msg.data.data() + y * msg.step;
        if (!depth) std::memcpy(output, row, msg.step);
        else {
            for (unsigned x=0; x<msg.width; ++x) {
                uint16_t raw;
                std::memcpy(&raw, row + 2*x, sizeof(raw));
                const auto mm = static_cast<uint16_t>(std::min(65535.0,
                    std::max(0.0, std::round(raw * depth_scale * 1000.0))));
                output[2*x] = mm & 255; output[2*x+1] = mm >> 8;
            }
        }
    }
    return msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "realsense_images");
    ros::NodeHandle nh, priv("~");
    try {
        if (ros::Time::isSimTime()) throw std::runtime_error("Live cameras require use_sim_time=false");
        std::string serial, prefix;
        int width, height, fps;
        if (!priv.getParam("serial_no", serial) || !priv.getParam("frame_prefix", prefix))
            throw std::runtime_error("serial_no and frame_prefix are required strings");
        priv.param("width",width,640); priv.param("height",height,480); priv.param("fps",fps,15);
        nh.deleteParam("calibration");
        rs2::pipeline pipeline;
        rs2::config config;
        config.enable_device(serial);
        config.enable_stream(RS2_STREAM_COLOR,width,height,RS2_FORMAT_RGB8,fps);
        config.enable_stream(RS2_STREAM_DEPTH,width,height,RS2_FORMAT_Z16,fps);
        auto device = config.resolve(pipeline).get_device();
        if (serial != device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER))
            throw std::runtime_error("Camera serial mismatch");
        for (auto sensor : device.query_sensors()) {
            if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
                sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED,1);
        }
        auto profile = pipeline.start(config);
        const double scale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
        if (!std::isfinite(scale) || scale<=0) throw std::runtime_error("Invalid depth scale");
        const auto color_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        const auto depth_profile = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        XmlRpc::XmlRpcValue calibration;
        calibration["serial"] = serial;
        calibration["color"] = intrinsics(color_profile);
        calibration["depth"] = intrinsics(depth_profile);
        calibration["sdk_depth_unit_m"] = scale;
        calibration["published_depth_unit_m"] = 0.001;
        calibration["timestamp_source"] = std::string("SDK global/system clock, milliseconds converted to ROS seconds");
        const auto extr = depth_profile.get_extrinsics_to(color_profile);
        for (int k=0; k<9; ++k) calibration["depth_to_color"]["rotation_column_major"][k] = double(extr.rotation[k]);
        for (int k=0; k<3; ++k) calibration["depth_to_color"]["translation_m"][k] = double(extr.translation[k]);
        auto color_pub = nh.advertise<sensor_msgs::Image>("color/image_raw",2);
        auto depth_pub = nh.advertise<sensor_msgs::Image>("depth/image_rect_raw",2);
        auto aligned_pub = nh.advertise<sensor_msgs::Image>("aligned_depth_to_color/image_raw",2);
        rs2::align align(RS2_STREAM_COLOR);
        ros::Time last_color, last_depth;
        auto last_valid = ros::WallTime::now();
        bool calibrated = false;
        ROS_INFO("%s serial=%s: publishing exactly RGB, depth and aligned depth",prefix.c_str(),serial.c_str());
        while (ros::ok()) {
            rs2::frameset frames;
            if ((ros::WallTime::now()-last_valid).toSec()>10)
                throw std::runtime_error("No fresh synchronized RGB/depth frames for 10 seconds");
            if (!pipeline.try_wait_for_frames(&frames,1000)) continue;
            const auto color = frames.get_color_frame();
            const auto depth = frames.get_depth_frame();
            if (!color || !depth) continue;
            ros::Time color_stamp, depth_stamp;
            if (!source_stamp(color,color_stamp) || !source_stamp(depth,depth_stamp)) {
                ROS_WARN_THROTTLE(2,"Waiting for valid SDK global/system timestamps");
                continue;
            }
            // SDK synchronization may reuse a frame. Do not manufacture new samples.
            if (color_stamp<=last_color || depth_stamp<=last_depth) continue;
            const auto aligned = align.process(frames).as<rs2::frameset>().get_depth_frame();
            if (!aligned) continue;
            if (!calibrated) {
                calibration["aligned_depth_to_color"] = intrinsics(aligned.get_profile().as<rs2::video_stream_profile>());
                nh.setParam("calibration",calibration);
                calibrated = true;
            }
            color_pub.publish(image_message(color,color_stamp,prefix+"_color_optical_frame",scale));
            depth_pub.publish(image_message(depth,depth_stamp,prefix+"_depth_optical_frame",scale));
            aligned_pub.publish(image_message(aligned,depth_stamp,prefix+"_color_optical_frame",scale));
            last_color=color_stamp; last_depth=depth_stamp; last_valid=ros::WallTime::now();
            ros::spinOnce();
        }
        pipeline.stop();
    } catch (const std::exception& e) {
        ROS_FATAL("Camera acquisition failed: %s",e.what());
        return 1;
    }
    return 0;
}
