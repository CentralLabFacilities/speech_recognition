#pragma ONCE

#include <ros/ros.h>
#include "parakeet/parakeet_capi.h"

#include <audio_common_msgs/AudioData.h>
#include <audio_common_msgs/AudioInfo.h>
#include <clf_speech_msgs/ASR.h>

class ParakeetRos
{
    public:
        ParakeetRos(ros::NodeHandle nh, std::string model);
        ~ParakeetRos();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_audio_data_;
        ros::Subscriber sub_audio_info_;
        void audioDataCallback(const audio_common_msgs::AudioDataConstPtr& msg);
        void audioInfoCallback(const audio_common_msgs::AudioInfoConstPtr& msg);
        ros::Publisher pub_asr_;

        audio_common_msgs::AudioInfo last_audio_info_;

        parakeet_ctx* ctx_;
};