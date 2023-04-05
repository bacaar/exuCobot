/*
C++ program to locate active VR devices and print their position to screen
Does no rendering.

Author: Aaron Bacher
        utility functions taken from https://github.com/matinas/openvrsimplexamples
*/

#include <iostream>
#include <vector>
#include <string>
#include <openvr/openvr.h>

// OpenVR variables 
vr::IVRSystem* vr_context;
vr::TrackedDevicePose_t tracked_device_pose[vr::k_unMaxTrackedDeviceCount];

// app variables
std::string driver_name, driver_serial;
std::string tracked_device_type[vr::k_unMaxTrackedDeviceCount];

// app functions (taken from https://github.com/matinas/openvrsimplexamples)
int init_OpenVR();
void process_vr_event(const vr::VREvent_t & event);
std::string GetTrackedDeviceString(vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError=NULL);
std::string GetTrackedDeviceClassString(vr::ETrackedDeviceClass td_class);


int main(int argv, char** args) {

	if (init_OpenVR() != 0) return -1;
	else std::cout << "OpenVR initialized successfully\n" << std::endl;

	double poseAccumulator[vr::k_unMaxTrackedDeviceCount][3];	// received poses will be summed up here
	int poseCounter[vr::k_unMaxTrackedDeviceCount];	// variable to count how many poses have been received per device
	
	// initialize all values to 0
	for(int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice){
		poseCounter[nDevice] = 0;
		for(int axis = 0; axis < 3; ++axis){
			poseAccumulator[nDevice][axis] = 0.0;
		}
	}

	for(int i = 0; i < 10000; ++i)
	{
		if (vr_context != NULL)
		{
			// Process SteamVR events
			vr::VREvent_t vr_event;
			while(vr_context->PollNextEvent(&vr_event,sizeof(vr_event)))
				process_vr_event(vr_event);

			// Obtain tracking device poses
			vr_context->GetDeviceToAbsoluteTrackingPose(vr::ETrackingUniverseOrigin::TrackingUniverseStanding,0,tracked_device_pose,vr::k_unMaxTrackedDeviceCount);

			for (int nDevice=0; nDevice<vr::k_unMaxTrackedDeviceCount; nDevice++)
			{
				if ((tracked_device_pose[nDevice].bDeviceIsConnected) && (tracked_device_pose[nDevice].bPoseIsValid))
				{
					// We take just the translation part of the matrix (actual position of tracked device, not orientation)
					float v[3] = { tracked_device_pose[nDevice].mDeviceToAbsoluteTracking.m[0][3], tracked_device_pose[nDevice].mDeviceToAbsoluteTracking.m[1][3], tracked_device_pose[nDevice].mDeviceToAbsoluteTracking.m[2][3]};

					// add received pose to accumulator
					for(int axis = 0; axis < 3; ++axis){
						poseAccumulator[nDevice][axis] += v[axis];
					}

					++poseCounter[nDevice];
				}
			}
		}
	}

	for(int nDevice=0; nDevice<vr::k_unMaxTrackedDeviceCount; nDevice++){
		if(poseCounter[nDevice] > 0){
			std::cout << "Got " << poseCounter[nDevice] << " values for device " << nDevice << " of type " << tracked_device_type[nDevice] << std::endl;
			// calculate and output mean values
			std::cout << "x=" << poseAccumulator[nDevice][0] / poseCounter[nDevice] << "\t";
			std::cout << "y=" << poseAccumulator[nDevice][1] / poseCounter[nDevice] << "\t";
			std::cout << "z=" << poseAccumulator[nDevice][2] / poseCounter[nDevice] << "\n";
		}
	}

	vr::VR_Shutdown();
	return 0;
}

int init_OpenVR()
{
	// Check whether there is an HMD plugged-in and the SteamVR runtime is installed
	if (vr::VR_IsHmdPresent())
	{
		std::cout << "An HMD was successfully found in the system" << std::endl;

		if (vr::VR_IsRuntimeInstalled()) {
			uint pathSize = 512;
			uint requiredPathSize = 512;
			char* path = new char[pathSize];
			bool success = vr::VR_GetRuntimePath(path, pathSize, &requiredPathSize);
			if (success == false)
			{
				std::cout << "Runtime not found\n";
			}

			delete[] path;
		}
		else
		{
			std::cout << "Runtime was not found, quitting app" << std::endl;
			return -1;
		}
	}
	else
	{
		std::cout << "No HMD was found in the system, quitting app" << std::endl;
		return -1;
	}

	// Load the SteamVR Runtime
	vr::HmdError err;
	vr_context = vr::VR_Init(&err,vr::EVRApplicationType::VRApplication_Scene);
	if(vr_context == NULL){
		std::cout << "Error while initializing SteamVR runtime. Error code is " << vr::VR_GetVRInitErrorAsSymbol(err) << std::endl;
		exit(-1);
	}
	else{
		std::cout << "SteamVR runtime successfully initialized" << std::endl;
	}

	// Obtain some basic information given by the runtime
	int base_stations_count = 0;
	for (uint32_t td=vr::k_unTrackedDeviceIndex_Hmd; td<vr::k_unMaxTrackedDeviceCount; td++) {

		if (vr_context->IsTrackedDeviceConnected(td))
		{
			vr::ETrackedDeviceClass tracked_device_class = vr_context->GetTrackedDeviceClass(td);

			std::string td_type = GetTrackedDeviceClassString(tracked_device_class);
			tracked_device_type[td] = td_type;

			if (tracked_device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference) base_stations_count++;

			if (td == vr::k_unTrackedDeviceIndex_Hmd)
			{
				// Fill variables used for obtaining the device name and serial ID (used later for naming the SDL window)
				driver_name = GetTrackedDeviceString(vr_context,vr::k_unTrackedDeviceIndex_Hmd,vr::Prop_TrackingSystemName_String);
				driver_serial = GetTrackedDeviceString(vr_context,vr::k_unTrackedDeviceIndex_Hmd,vr::Prop_SerialNumber_String);
			}
		}
	}

	// Check whether both base stations are found, not mandatory but just in case...
	if (base_stations_count < 2)
	{
		std::cout << "There was a problem indentifying the base stations, please check they are powered on" << std::endl;

		return -1;
	}

	return 0;
}

// register and log device updates
void process_vr_event(const vr::VREvent_t & event)
{
	std::string str_td_class = GetTrackedDeviceClassString(vr_context->GetTrackedDeviceClass(event.trackedDeviceIndex));

	switch(event.eventType)
	{
	case vr::VREvent_TrackedDeviceActivated:
		{
			std::cout << "Device " << event.trackedDeviceIndex << " attached (" << str_td_class << ")" << std::endl;
			tracked_device_type[event.trackedDeviceIndex] = str_td_class;
		}
		break;
	case vr::VREvent_TrackedDeviceDeactivated:
		{
			std::cout << "Device " << event.trackedDeviceIndex << " detached (" << str_td_class << ")" << std::endl;
			tracked_device_type[event.trackedDeviceIndex] = "";
		}
		break;
	case vr::VREvent_TrackedDeviceUpdated:
		{
			std::cout << "Device " << event.trackedDeviceIndex << " updated (" << str_td_class << ")" << std::endl;
		}
		break;
	}
}

// function to get a string from a tracked device property and turn it into a std::string
std::string GetTrackedDeviceString(vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError)
{
	uint32_t requiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
	if( requiredBufferLen == 0 )
		return "";

	char *pchBuffer = new char[requiredBufferLen];
	requiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice,prop,pchBuffer,requiredBufferLen,peError);
	std::string sResult = pchBuffer;
	delete[] pchBuffer;

	return sResult;
}

// function to get a string from a tracked device type class
std::string GetTrackedDeviceClassString(vr::ETrackedDeviceClass td_class) {

	std::string str_td_class = "Unknown class";

	switch (td_class)
	{
	case vr::TrackedDeviceClass_Invalid:			// = 0, the ID was not valid.
		str_td_class = "invalid";
		break;
	case vr::TrackedDeviceClass_HMD:				// = 1, Head-Mounted Displays
		str_td_class = "hmd";
		break;
	case vr::TrackedDeviceClass_Controller:			// = 2, Tracked controllers
		str_td_class = "controller";
		break;
	case vr::TrackedDeviceClass_GenericTracker:		// = 3, Generic trackers, similar to controllers
		str_td_class = "generic tracker";
		break;
	case vr::TrackedDeviceClass_TrackingReference:	// = 4, Camera and base stations that serve as tracking reference points
		str_td_class = "base station";
		break;
	case vr::TrackedDeviceClass_DisplayRedirect:	// = 5, Accessories that aren't necessarily tracked themselves, but may redirect video output from other tracked devices
		str_td_class = "display redirect";
		break;
	}

	return str_td_class;
}