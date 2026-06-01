#include <ros_parakeetcpp/parakeet.hpp>

#include <chrono>

ParakeetRos::ParakeetRos(ros::NodeHandle nh, std::string model) : nh_(nh) {

    ctx_ = parakeet_capi_load(model.c_str());
    if (!ctx_) { 
        ros::shutdown();
    }

    sub_audio_data_ = nh_.subscribe<audio_common_msgs::AudioData>(
        "input",
        1, 
        &ParakeetRos::audioDataCallback, 
        this
    );

    sub_audio_info_ = nh_.subscribe<audio_common_msgs::AudioInfo>(
        "/audio_info", 
        1, 
        &ParakeetRos::audioInfoCallback, 
        this
    );

    pub_asr_ = nh_.advertise<clf_speech_msgs::ASR>("asr", 10);

    ROS_INFO("ParakeetRos initialized.");
}

ParakeetRos::~ParakeetRos(){}


void ParakeetRos::audioInfoCallback(const audio_common_msgs::AudioInfoConstPtr& msg) {
    //last_audio_info_ = msg;
}


void ParakeetRos::audioDataCallback(const audio_common_msgs::AudioDataConstPtr& msg) {
    //ROS_INFO_THROTTLE(1.0, "AudioData callback");
    auto start = std::chrono::high_resolution_clock::now();

    int n_samples = static_cast<int>(msg->data.size() / sizeof(float));
    const uint8_t* raw_bytes = msg->data.data();
    const float* samples = reinterpret_cast<const float*>(raw_bytes);
    char* transcript = parakeet_capi_transcribe_pcm(ctx_, samples, n_samples, 16000, 0);


    auto end = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

    if (transcript != nullptr) {
        clf_speech_msgs::ASR result;
        result.text = transcript;
        ROS_INFO_STREAM("speech result: '" << result.text << "' took: " << elapsed_ms << "ms");
        pub_asr_.publish(result);
        parakeet_capi_free_string(transcript);
    } else {
        const char* err = parakeet_capi_last_error(ctx_);
        ROS_ERROR_THROTTLE(1.0, "Transcription failed: %s", err ? err : "Unknown error");
    }
}