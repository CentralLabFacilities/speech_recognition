respeaker_ros
=============

A ROS Package for Respeaker Mic Array


## Supported Devices

- [Respeaker Mic Array v2.0](http://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/)
    ![Respeaker Mic Array v2.0](https://github.com/SeeedDocument/ReSpeaker_Mic_Array_V2/raw/master/img/Hardware%20Overview.png)

## Preparation

1. Register respeaker udev rules

    Normally, we cannot access USB device without permission from user space.
    Using `udev`, we can give the right permission on only respeaker device automatically.

    Please run the command as followings to install setting file:

    ```bash
    roscd respeaker_ros
    sudo cp -f $(rospack find respeaker_ros)/config/60-respeaker.rules /etc/udev/rules.d/60-respeaker.rules
    sudo systemctl restart udev
    ```

    And then re-connect the device.

1. Update firmware

    ```bash
    git clone https://github.com/respeaker/usb_4_mic_array.git
    cd usb_4_mic_array
    sudo python dfu.py --download 6_channels_firmware.bin  # The 6 channels version 
    ```

## How to use

1. Run executables

    ```bash
    roslaunch respeaker_ros respeaker.launch
    rostopic echo /sound_direction     # Result of DoA
    rostopic echo /sound_localization  # Result of DoA as Pose
    rostopic echo /is_speeching        # Result of VAD
    rostopic echo /audio               # Raw audio
    rostopic echo /speech_audio        # Audio data while speeching
    ```

    You can also set various parameters via `dynamic_reconfigure`.


## Use cases

## Notes

The configuration file for `dynamic_reconfigure` in this package is created automatically by reading the parameters from devices.
Though it will be rare case, the configuration file can be updated as followings:

1. Connect the device to the computer.
1. Run the generator script.

    ```bash
    rosrun  respeaker_ros respeaker_<DEVICE>_gencfg.py
    ```
1. You will see the updated configuration file at `$(rospack find respeaker_ros)/cfg/Respeaker<DEVICE>.cfg`.

## License

[Apache License](LICENSE)
