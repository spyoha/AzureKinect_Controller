#include "Kinect_Manager.h"


Kinect_Manager::Kinect_Manager(uint32_t _device_index, bool _use_IMU) :
	m_kinect_id(UINT32_MAX),
	m_is_initialized(false),
	m_running_cameras(false),
	m_running_IMU(false)
{
	m_kinect_id = _device_index;
	
	if (true == Initialize_Kinect_Device())
		m_is_initialized = true;

	Set_Device_Config();
}


Kinect_Manager::~Kinect_Manager()
{
	Release_Kinect_Device();
}


void Kinect_Manager::Set_Device_Config(k4a_device_configuration_t & _config)
{
	Stop_All_Devices();

	m_kinect_config = _config;
}

void Kinect_Manager::Set_Device_Config(k4a_image_format_t _color_format, k4a_color_resolution_t _color_resolution, k4a_depth_mode_t _depth_mode, k4a_fps_t _camera_fps, bool _synchronized_images_only, int32_t _depth_delay_off_color_usec, k4a_wired_sync_mode_t _wired_sync_mode, uint32_t _subordinate_delay_off_master_usec, bool _disable_streaming_indicator)
{
	Stop_All_Devices();

	m_kinect_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

	m_kinect_config.color_format = _color_format;
	m_kinect_config.color_resolution = _color_resolution;
	m_kinect_config.depth_mode = _depth_mode;
	m_kinect_config.camera_fps = _camera_fps;
	m_kinect_config.synchronized_images_only = _synchronized_images_only;
	m_kinect_config.depth_delay_off_color_usec = _depth_delay_off_color_usec;
	m_kinect_config.wired_sync_mode = _wired_sync_mode;
	m_kinect_config.subordinate_delay_off_master_usec = _subordinate_delay_off_master_usec;
	m_kinect_config.disable_streaming_indicator = _disable_streaming_indicator;
}

bool Kinect_Manager::Initialize_Kinect_Device()
{
	// check connected device
	if (0 >= k4a::device::get_installed_count())
		return false;

	// device ID initialized
	if (m_kinect_id == UINT32_MAX)
		return false;

	if (m_kinect_id >= k4a::device::get_installed_count())
		return false;


	m_kinect_device = k4a::device::open(m_kinect_id);

	return true;
}

std::chrono::microseconds Kinect_Manager::Get_TimeStamp_Camera()
{
	if (false == m_running_cameras)
		this->Start_Kinect_Cameras();

	k4a::capture* capture = new k4a::capture();

	if (false == m_kinect_device.get_capture(capture))
		return std::chrono::microseconds(-1);

	return capture->get_ir_image().get_device_timestamp();
}

std::chrono::microseconds Kinect_Manager::Get_TimeStamp_IMU()
{
	if (false == m_running_IMU)
		this->Start_Kinect_IMU();

	k4a_imu_sample_t* imu_sample = new k4a_imu_sample_t;

	if(false == m_kinect_device.get_imu_sample(imu_sample))
		return std::chrono::microseconds(-1);

	// acc_timestamp_usec == gyro_timestamp_usec
	return std::chrono::microseconds(imu_sample->acc_timestamp_usec);
}

void Kinect_Manager::Get_IMU_Sample(k4a_imu_sample_t* _imu_sample)
{
	if (false == m_running_IMU)
		this->Start_Kinect_IMU();

	m_kinect_device.get_imu_sample(_imu_sample);
}


bool Kinect_Manager::Release_Kinect_Device()
{
	Stop_All_Devices();

	m_kinect_device.close();

	return true;
}

void Kinect_Manager::Update_Calibration()
{
	m_kinect_calib = m_kinect_device.get_calibration(m_kinect_config.depth_mode, m_kinect_config.color_resolution);
}

void Kinect_Manager::Stop_All_Devices()
{
	if (false == m_is_initialized)
		return;

	if (true == m_running_IMU)
		Stop_Kinect_IMU();

	if (true == m_running_cameras)
		Stop_Kinect_Cameras();
}

bool Kinect_Manager::Start_Kinect_Cameras()
{
	if (true == m_running_cameras)
		return true;

	if (false == this->IsInitialized())
		return false;

	m_kinect_device.start_cameras(&m_kinect_config);

	m_running_cameras = true;

	return true;
}

bool Kinect_Manager::Stop_Kinect_Cameras()
{
	if (true != m_running_cameras)
		return true;

	m_kinect_device.stop_cameras();

	m_running_cameras = false;

	return true;
}

bool Kinect_Manager::Start_Kinect_IMU()
{
	if (true == m_running_IMU)
		return true;

	if (false == this->IsInitialized())
		return false;

	if (false == m_running_cameras)
		this->Start_Kinect_Cameras();

	m_kinect_device.start_imu();

	m_running_IMU = true;

	return true;
}

bool Kinect_Manager::Stop_Kinect_IMU()
{
	if (true != m_running_IMU)
		return true;

	m_kinect_device.stop_imu();

	m_running_IMU = false;

	return true;
}
