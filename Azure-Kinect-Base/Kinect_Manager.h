#pragma once

#include <k4a/k4a.hpp>



class Kinect_Manager
{
private:

	//////// Kinect device handler & setting
	uint32_t					m_kinect_id;
	k4a::device					m_kinect_device;
	k4a_device_configuration_t	m_kinect_config;
	k4a::calibration			m_kinect_calib;

	//////// Kinect running status
	bool			m_is_initialized;
	bool			m_running_cameras;
	bool			m_running_IMU;


public:
	Kinect_Manager(uint32_t _device_index = 0, bool _use_IMU = true);
	~Kinect_Manager();

	//////// Kinect device pre-configuration
	void	Set_Device_Config(k4a_device_configuration_t &_config);
	void	Set_Device_Config(
		k4a_image_format_t _color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32,
		k4a_color_resolution_t _color_resolution = K4A_COLOR_RESOLUTION_720P,
		k4a_depth_mode_t _depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED,
		k4a_fps_t _camera_fps = K4A_FRAMES_PER_SECOND_30,
		bool _synchronized_images_only = true,
		int32_t _depth_delay_off_color_usec = 0,
		k4a_wired_sync_mode_t _wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE,
		uint32_t _subordinate_delay_off_master_usec = 0,
		bool _disable_streaming_indicator = false
	);

	//////// Access Kinect configure & data

	// get the number of connected kinect devices 
	uint32_t	Get_Installed_Devices()	{ return k4a::device::get_installed_count(); }

	bool	IsInitialized()		{ return m_is_initialized; }
	bool	IsRunning_Camera()	{ return m_running_cameras; }
	bool	IsRunning_IMU()		{ return m_running_IMU; }

	// get Kinect sensor data
	std::chrono::microseconds Get_TimeStamp_Camera();
	std::chrono::microseconds Get_TimeStamp_IMU();

	void	Get_IMU_Sample(k4a_imu_sample_t* _imu_sample);



private:

	bool	Initialize_Kinect_Device();

	bool	Release_Kinect_Device();
	
	void	Update_Calibration();

	//////// Configure camera running status
	bool	Start_Kinect_Cameras();
	bool	Stop_Kinect_Cameras();

	bool	Start_Kinect_IMU();
	bool	Stop_Kinect_IMU();

	void	Stop_All_Devices();
};

