respeaker_ros
=============

A ROS Package for Respeaker Mic Array


## Supported Devices

- [Respeaker Mic Array v2.0](http://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/)
- [Respeaker xvf3800](https://wiki.seeedstudio.com/respeaker_xvf3800_introduction/)

## Preparation

1. Register respeaker udev rules

    Please run the command as followings to install:

    ```bash
    roscd respeaker_ros
    sudo cp -f $(rospack find respeaker_ros)/config/60-respeaker.rules /etc/udev/rules.d/60-respeaker.rules
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```

    And then re-connect the device.



## OLD DOC

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
