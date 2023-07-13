#include "DotPadManager.h"
#include "dot_pad_sdk_error.h"
#include <iostream>
#include <stdlib.h>
#include <Tchar.h>
#include <future>

using namespace DOT_PAD_SDK_API;

DotPadManager::DotPadManager(ros::NodeHandle nh, CString &COMPort) : nodeHandle_(nh)
{
    this->DOTPAD_COMPORT = COMPort;
    sub_data_ = nodeHandle_.subscribe(TOPIC_DOTPAD_DATA, 1, &DotPadManager::dataCallback, this);

    LoadDotPadDLL();

    dot_pad_register_key_callback(keyCallback);
    dot_pad_register_display_callback(screenCallback);

    initDotPad();

    ROS_INFO("DotPad is live");
}

DotPadManager::~DotPadManager()
{
    resetDisplay();
    FreeDotPadDLL();
}

void DotPadManager::LoadDotPadDLL()
{

    std::printf("Loading library... ");

    m_hDotPadDLL = LoadLibrary(_T("DotPadSDK.dll"));

    if (m_hDotPadDLL == NULL)
    {
        CString str;
        str.Format(_T("Failed! \t Error = (0x%x)"), GetLastError());
        std::cout << str;

        return;
    }
    else
        std::cout << "Success!\n";

    dot_pad_init = (DOT_PAD_INIT_FUNC)GetProcAddress(m_hDotPadDLL, "DOT_PAD_INIT");
    dot_pad_deinit = (DOT_PAD_DEINIT_FUNC)GetProcAddress(m_hDotPadDLL, "DOT_PAD_DEINIT");
    dot_pad_display = (DOT_PAD_DISPLAY_FUNC)GetProcAddress(m_hDotPadDLL, "DOT_PAD_DISPLAY");
    dot_pad_display_data = (DOT_PAD_DISPLAY_DATA_FUNC)GetProcAddress(m_hDotPadDLL, "DOT_PAD_DISPLAY_DATA");
    dot_pad_display_data_part = (DOT_PAD_DISPLAY_DATA_PART_FUNC)GetProcAddress(m_hDotPadDLL, "DOT_PAD_DISPLAY_DATA_PART");
    dot_pad_reset_display = (DOT_PAD_RESET_DISPLAY_FUNC)GetProcAddress(m_hDotPadDLL, "DOT_PAD_RESET_DISPLAY");
    dot_pad_braille_display = (DOT_PAD_BRAILLE_DISPLAY_FUNC)GetProcAddress(m_hDotPadDLL, "DOT_PAD_BRAILLE_DISPLAY");
    dot_pad_braille_ascii_display = (DOT_PAD_BRAILLE_ASCII_DISPLAY_FUNC)GetProcAddress(m_hDotPadDLL, "DOT_PAD_BRAILLE_ASCII_DISPLAY");
    dot_pad_reset_braille_display = (DOT_PAD_RESET_BRAILLE_DISPLAY_FUNC)GetProcAddress(m_hDotPadDLL, "DOT_PAD_RESET_BRAILLE_DISPLAY");
    dot_pad_send_key = (DOT_PAD_SEND_KEY_FUNC)GetProcAddress(m_hDotPadDLL, "DOT_PAD_SEND_KEY");
    dot_pad_get_fw_version = (DOT_PAD_GET_FW_VERSION_FUNC)GetProcAddress(m_hDotPadDLL, "DOT_PAD_GET_FW_VERSION");
    dot_pad_get_device_name = (DOT_PAD_GET_DEVICE_NAME_FUNC)GetProcAddress(m_hDotPadDLL, "DOT_PAD_GET_DEVICE_NAME");
    dot_pad_get_display_info = (DOT_PAD_GET_DISPLAY_INFO_FUNC)GetProcAddress(m_hDotPadDLL, "DOT_PAD_GET_DISPLAY_INFO");
    dot_pad_register_key_callback = (DOT_PAD_REGISTER_KEY_CALLBACK_FUNC)GetProcAddress(m_hDotPadDLL, "DOT_PAD_REGISTER_KEY_CALLBACK");
    dot_pad_register_display_callback = (DOT_PAD_REGISTER_DISPLAY_CALLBACK_FUNC)GetProcAddress(m_hDotPadDLL, "DOT_PAD_REGISTER_DISPLAY_CALLBACK");

    if (dot_pad_init == NULL)
    {
        std::cout << _T("Can not find funtion DOT_PAD_INIT_FUNC in DotPadSDK.dll!");
    }
    if (dot_pad_deinit == NULL)
    {
        std::cout << _T("Can not find funtion DOT_PAD_DEINIT_FUNC in DotPadSDK.dll!");
    }
    if (dot_pad_display == NULL)
    {
        std::cout << _T("Can not find funtion DOT_PAD_DISPLAY_FUNC in DotPadSDK.dll!");
    }
    if (dot_pad_display_data == NULL)
    {
        std::cout << _T("Can not find funtion DOT_PAD_DISPLAY_DATA_FUNC in DotPadSDK.dll!");
    }
    if (dot_pad_display_data_part == NULL)
    {
        std::cout << _T("Can not find funtion DOT_PAD_DISPLAY_DATA_PART_FUNC in DotPadSDK.dll!");
    }
    if (dot_pad_reset_display == NULL)
    {
        std::cout << _T("Can not find funtion DOT_PAD_RESET_DISPLAY_FUNC in DotPadSDK.dll!");
    }
    if (dot_pad_braille_display == NULL)
    {
        std::cout << _T("Can not find funtion DOT_PAD_BRAILLE_DISPLAY_FUNC in DotPadSDK.dll!");
    }
    if (dot_pad_braille_ascii_display == NULL)
    {
        std::cout << _T("Can not find funtion DOT_PAD_BRAILLE_ASCII_DISPLAY_FUNC in DotPadSDK.dll!");
    }
    if (dot_pad_reset_braille_display == NULL)
    {
        std::cout << _T("Can not find funtion DOT_PAD_RESET_BRAILLE_DISPLAY_FUNC in DotPadSDK.dll!");
    }
    if (dot_pad_send_key == NULL)
    {
        std::cout << _T("Can not find funtion DOT_PAD_SEND_KEY_FUNC in DotPadSDK.dll!");
    }
    if (dot_pad_get_fw_version == NULL)
    {
        std::cout << _T("Can not find funtion DOT_PAD_GET_FW_VERSION in DotPadSDK.dll!");
    }
    if (dot_pad_get_device_name == NULL)
    {
        std::cout << _T("Can not find funtion DOT_PAD_GET_DEVICE_NAME in DotPadSDK.dll!");
    }
    if (dot_pad_get_display_info == NULL)
    {
        std::cout << _T("Can not find funtion DOT_PAD_GET_DISPLAY_INFO_FUNC in DotPadSDK.dll!");
    }
    if (dot_pad_register_key_callback == NULL)
    {
        std::cout << _T("Can not find funtion DOT_PAD_REGISTER_KEY_CALLBACK_FUNC in DotPadSDK.dll!");
    }
    if (dot_pad_register_display_callback == NULL)
    {
        std::cout << _T("Can not find funtion DOT_PAD_REGISTER_DISPLAY_CALLBACK_FUNC in DotPadSDK.dll!");
    }
}

void DotPadManager::FreeDotPadDLL()
{

    std::printf("Freeing DotPad DLL... ");

    if (m_hDotPadDLL)
    {
        FreeLibrary(m_hDotPadDLL);
        m_hDotPadDLL = NULL;
    }

    dot_pad_init = NULL;
    dot_pad_deinit = NULL;
    dot_pad_display = NULL;
    dot_pad_display_data = NULL;
    dot_pad_display_data_part = NULL;
    dot_pad_reset_display = NULL;
    dot_pad_braille_display = NULL;
    dot_pad_braille_ascii_display = NULL;
    dot_pad_reset_braille_display = NULL;
    dot_pad_send_key = NULL;
    dot_pad_get_fw_version = NULL;
    dot_pad_get_device_name = NULL;
    dot_pad_register_key_callback = NULL;
    dot_pad_register_display_callback = NULL;

    std::cout << "Success! \n";
}

void DotPadManager::initDotPad()
{

    std::printf("DotPad Initialization ... ");

    CString str;

    DOT_PAD_SDK_ERROR error = DOT_ERROR_COM_PORT_ERROR;

    do
    {
        error = dot_pad_init(_ttoi(this->DOTPAD_COMPORT));

        if (error == DOT_ERROR_NONE)
        {
            str = "Success! \n";
            std::cout << str;
        }

        else
        {
            str.Format(_T("Failed with error (0x%x), waiting %ds to retry\n"), error, INIT_TIMEOUT_SECONDS);
            std::cout << str;

            _sleep(INIT_TIMEOUT_SECONDS * 1000);
        }

    } while (error != DOT_ERROR_NONE);
}

void DotPadManager::resetDisplay()
{

    std::printf("Resetting Display");

    dot_pad_reset_display();
    std::future<void> myFuture = std::async(screenCallback);

    std::printf("Display Reset!\n");
}

DOT_PAD_SDK_ERROR DotPadManager::displayData(uint8_t *data, int dataLength)
{

    DOT_PAD_SDK_ERROR error = dot_pad_display_data(data, dataLength, false);

    return error;
}

void DotPadManager::dataCallback(const std_msgs::UInt8MultiArray::ConstPtr &message)
{

    // Got uint8_t multiarray message from ROS server
    // Check if message is of the correct layout

    std_msgs::MultiArrayLayout layout = message->layout;

    if (layout.dim[0].size > 300)
    {
        ROS_ERROR("Received data too large for display onscreen: %d > 300", layout.dim[0].size);
    }

    // Data is of correct size, cast to uint8_t array and try to plot to screen
    ROS_INFO("got data: dim[0].size=%d", layout.dim[0].size);

    // Take only first 300 elements of array
    std::vector<uint8_t> message_data(300);
    message_data.insert(message_data.begin(), message->data.begin(), message->data.begin() + 299);

    // message->data is of type std::vector<uint8_t>, to convert point to first value in array
    uint8_t *dataForScreen;
    dataForScreen = &message_data[0];

    DOT_PAD_SDK_ERROR error = displayData(dataForScreen, 300);

    if (error != DOT_ERROR_NONE)
    {
        switch (error)
        {

        case DOT_ERROR_DISPLAY_DATA_UNCHAGNED:
            ROS_ERROR("Display data is same!!");
            break;

        case DOT_ERROR_COM_WRITE_ERROR:
            ROS_ERROR("COM Write error");
            break;

        case DOT_ERROR_COM_INVALID_DATA:
            ROS_ERROR("Invalid data");
            break;

        case DOT_ERROR_COM_NOT_RESPONSE:
            ROS_ERROR("No response from COM port");
            break;

        case DOT_ERROR_DISPLAY_DATA_RANGE_INVALID:
            ROS_ERROR("Invalid data range");
            break;

        case DOT_ERROR_DISPLAY_IN_PROGRESS:
            ROS_ERROR("Display is busy, cannot update yet!");
            break;

        default:
            ROS_ERROR("Display failed with error (0x%X)\n", error);
            break;
        }
    }

    else
    {
        ROS_INFO("Printed succesfully!");
    }
}

void CALLBACK screenCallback(void)
{
    // printing = false;
    ROS_INFO("Display complete");
}

void CALLBACK keyCallback(const int key)
{
    ROS_INFO("Key pressed: %d", key);
}