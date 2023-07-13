#pragma once

#include <ros/ros.h>
#include <string>
#include <atlstr.h>
#include <windows.h>
#include <dot_pad_sdk.h>
#include <dot_pad_sdk_error.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Bool.h>

const int INIT_TIMEOUT_SECONDS = 5;
const std::string TOPIC_DOTPAD_DATA = "/mapper/output_data";
const std::string TOPIC_SCREEN_STATUS = "/dotpad/screen_finished";

using namespace DOT_PAD_SDK_API;

class DotPadManager
{
public:
    DotPadManager(ros::NodeHandle nh, CString &COMPort); // Constructor
    ~DotPadManager();                                    // Destructor

    DOT_PAD_SDK_ERROR displayData(uint8_t *data, int dataLength);
    void resetDisplay();

private:
    ros::NodeHandle nodeHandle_;
    ros::Subscriber sub_data_;
    CString DOTPAD_COMPORT;

    void LoadDotPadDLL();
    void FreeDotPadDLL();

private:
    bool printing;

    HINSTANCE m_hDotPadDLL;
    DOT_PAD_INIT_FUNC dot_pad_init;
    DOT_PAD_DEINIT_FUNC dot_pad_deinit;
    DOT_PAD_DISPLAY_FUNC dot_pad_display;
    DOT_PAD_DISPLAY_DATA_FUNC dot_pad_display_data;
    DOT_PAD_DISPLAY_DATA_PART_FUNC dot_pad_display_data_part;
    DOT_PAD_RESET_DISPLAY_FUNC dot_pad_reset_display;
    DOT_PAD_BRAILLE_DISPLAY_FUNC dot_pad_braille_display;
    DOT_PAD_BRAILLE_ASCII_DISPLAY_FUNC dot_pad_braille_ascii_display;
    DOT_PAD_RESET_BRAILLE_DISPLAY_FUNC dot_pad_reset_braille_display;
    DOT_PAD_SEND_KEY_FUNC dot_pad_send_key;
    DOT_PAD_GET_FW_VERSION_FUNC dot_pad_get_fw_version;
    DOT_PAD_GET_DEVICE_NAME_FUNC dot_pad_get_device_name;
    DOT_PAD_GET_DISPLAY_INFO_FUNC dot_pad_get_display_info;
    DOT_PAD_REGISTER_KEY_CALLBACK_FUNC dot_pad_register_key_callback;
    DOT_PAD_REGISTER_DISPLAY_CALLBACK_FUNC dot_pad_register_display_callback;

    void initDotPad();
    void dataCallback(const std_msgs::UInt8MultiArray::ConstPtr &message);
};

void CALLBACK screenCallback();
void CALLBACK keyCallback(const int key);
