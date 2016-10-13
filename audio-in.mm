// g++ -framework AudioUnit -framework AudioToolbox -framework CoreAudio -framework CoreFoundation -framework Foundation -o au -Wall audio-in.mm

#include <AudioUnit/AudioUnit.h>
#include <AudioToolbox/AudioToolbox.h>
#include <CoreAudio/CoreAudio.h>
#include <CoreFoundation/CoreFoundation.h>

#define max(a, b) (((a) > (b)) ? (a) : (b))
#define min(a, b) (((a) < (b)) ? (a) : (b))

#define kOutputBus 0
#define kInputBus 1

typedef struct data {
	AudioUnit& AU;
	AudioBufferList inputBuffer;
} data;

void checkStatus(OSStatus status, const char *func)
{
	if ((int)status == 0)
		return;

	const char *serr;

	switch(status) {
	case kAudioUnitErr_InvalidProperty:
		serr = "kAudioUnitErr_InvalidProperty";
		break;
	case kAudioUnitErr_InvalidParameter:
		serr = "kAudioUnitErr_InvalidParameter";
		break;
	case kAudioUnitErr_InvalidElement:
		serr = "kAudioUnitErr_InvalidElement";
		break;
	case kAudioUnitErr_NoConnection:
		serr = "kAudioUnitErr_NoConnection";
		break;
	case kAudioUnitErr_FailedInitialization:
		serr = "kAudioUnitErr_FailedInitialization";
		break;
	case kAudioUnitErr_TooManyFramesToProcess:
		serr = "kAudioUnitErr_TooManyFramesToProcess";
		break;
	case kAudioUnitErr_InvalidFile:
		serr = "kAudioUnitErr_InvalidFile";
		break;
	case kAudioUnitErr_UnknownFileType:
		serr = "kAudioUnitErr_UnknownFileType";
		break;
	case kAudioUnitErr_FileNotSpecified:
		serr = "kAudioUnitErr_FileNotSpecified";
		break;
	case kAudioUnitErr_FormatNotSupported:
		serr = "kAudioUnitErr_FormatNotSupported";
		break;
	case kAudioUnitErr_Uninitialized:
		serr = "kAudioUnitErr_Uninitialized";
		break;
	case kAudioUnitErr_InvalidScope:
		serr = "kAudioUnitErr_InvalidScope";
		break;
	case kAudioUnitErr_PropertyNotWritable:
		serr = "kAudioUnitErr_PropertyNotWritable";
		break;
	case kAudioUnitErr_CannotDoInCurrentContext:
		serr = "kAudioUnitErr_CannotDoInCurrentContext";
		break;
	case kAudioUnitErr_InvalidPropertyValue:
		serr = "kAudioUnitErr_InvalidPropertyValue";
		break;
	case kAudioUnitErr_PropertyNotInUse:
		serr = "kAudioUnitErr_PropertyNotInUse";
		break;
	case kAudioUnitErr_Initialized:
		serr = "kAudioUnitErr_Initialized";
		break;
	case kAudioUnitErr_InvalidOfflineRender:
		serr = "kAudioUnitErr_InvalidOfflineRender";
		break;
	case kAudioUnitErr_Unauthorized:
		serr = "kAudioUnitErr_Unauthorized";
		break;
	default:
		serr = "Unknown_error";
	}

	fprintf(stderr, "[%s]: %s (%d)\n", func, serr, (int)status);
}

/*
	mic -> Input_Scope    / kInputBus /    Output_Scope -> app
	hph <- Output_Scope   / kOutputBus /   Input_Scope  <- app
*/

static OSStatus recordingCallback(void *inRefCon,
		  AudioUnitRenderActionFlags *ioActionFlags,
		  const AudioTimeStamp *inTimeStamp,
		  UInt32 inBusNumber,
		  UInt32 inNumberFrames,
		  AudioBufferList *ioData) {

	fprintf(stdout, ">>> recordingCallback: frames = %u, bus = %u\n", inNumberFrames, inBusNumber);
	return noErr;
}
/*
	OSStatus status;
	AudioComponentInstance audioUnit = (AudioComponentInstance) inRefCon;
	status = AudioUnitRender(audioUnit,
				ioActionFlags,
				inTimeStamp,
				inBusNumber,
				inNumberFrames,
				&inputBuffer);
	checkStatus(status, "AudioUnitRender");

	return noErr;
}
*/

void set_default_device(AudioUnit &AU)
{
	OSStatus status;

	AudioDeviceID defaultDevice = kAudioObjectUnknown;
	UInt32 propertySize = sizeof (defaultDevice);
	AudioObjectPropertyAddress defaultDeviceProperty;
	defaultDeviceProperty.mSelector = kAudioHardwarePropertyDefaultInputDevice;
	defaultDeviceProperty.mScope = kAudioObjectPropertyScopeGlobal;
	defaultDeviceProperty.mElement = kAudioObjectPropertyElementMaster;

	status = AudioObjectGetPropertyData(kAudioObjectSystemObject,
			&defaultDeviceProperty,
			0,
			NULL,
			&propertySize,
			&defaultDevice);
	checkStatus(status, "AudioObjectGetPropertyData DefaultInputDevice");

	status = AudioUnitSetProperty(AU,
			kAudioOutputUnitProperty_CurrentDevice,
			kAudioUnitScope_Global,
			kInputBus,
			&defaultDevice,
			sizeof(defaultDevice));
	checkStatus(status, "AudioUnitSetProperty CurrentDevice");
}

void set_format(AudioUnit& AU, bool input)
{
	OSStatus status;
	// Get audio format
	AudioStreamBasicDescription AudioFormat;
	UInt32 size = sizeof(AudioFormat);
	status = AudioUnitGetProperty(AU,
			kAudioUnitProperty_StreamFormat,
			input? kAudioUnitScope_Input : kAudioUnitScope_Output,
			input ? kInputBus : kOutputBus,
			&AudioFormat,
			&size);
	checkStatus(status, "AudioUnitGetProperty StreamFormat");

	fprintf(stdout, "%s - AudioFormat: \n\t"
                "format_id=%d\n\tformat_flags=%x\n\tframes_per_pack=%d\n\t"
                "channels_per_frame=%d\n\tbits_per_channel=%d\n\tsample_rate=%lf\n",
		input ? "in" : "out",
		AudioFormat.mFormatID,
		AudioFormat.mFormatFlags,
		AudioFormat.mFramesPerPacket,
		AudioFormat.mChannelsPerFrame,
		AudioFormat.mBitsPerChannel,
		AudioFormat.mSampleRate);

	// Set audio format

	AudioFormat.mReserved		= 0;
	AudioFormat.mFormatID		= kAudioFormatLinearPCM;
	AudioFormat.mFormatFlags	= kAudioFormatFlagIsFloat | kAudioFormatFlagsNativeEndian | kAudioFormatFlagIsPacked;
	AudioFormat.mBitsPerChannel	= 32;
	AudioFormat.mFramesPerPacket	= 1;
	AudioFormat.mBytesPerFrame	= AudioFormat.mBitsPerChannel * AudioFormat.mChannelsPerFrame / 8;
	AudioFormat.mBytesPerPacket	= AudioFormat.mBytesPerFrame * AudioFormat.mFramesPerPacket;

	status = AudioUnitSetProperty(AU,
			kAudioUnitProperty_StreamFormat,
			input? kAudioUnitScope_Output : kAudioUnitScope_Input,
			input ? kInputBus : kOutputBus,
			&AudioFormat,
			sizeof(AudioFormat));
	checkStatus(status, "AudioUnitSetProperty StreamFormat Scope_In/Output");
}

/*
	mic -> Input_Scope    / kInputBus /    Output_Scope -> app
	hph <- Output_Scope   / kOutputBus /   Input_Scope  <- app
*/

int main(int argc, char **argv)
{
	fprintf(stdout, "Tuning environment\n");

	OSStatus status;
	AudioUnit inAU;

	// Describe audio component
	AudioComponentDescription desc;
	desc.componentType = kAudioUnitType_Output;
	desc.componentSubType = kAudioUnitSubType_HALOutput;
	desc.componentManufacturer = kAudioUnitManufacturer_Apple;
	desc.componentFlags = 0;
	desc.componentFlagsMask = 0;

	// Get component
	AudioComponent component = AudioComponentFindNext(NULL, &desc);

	// Get audio units
	status = AudioComponentInstanceNew(component, &inAU);
	checkStatus(status, "AudioComponentInstanceNew in");

	// Set input callback
	AURenderCallbackStruct callbackStruct;
	callbackStruct.inputProc = recordingCallback;
	callbackStruct.inputProcRefCon = (void *)inAU;
	status = AudioUnitSetProperty(inAU,
				  kAudioOutputUnitProperty_SetInputCallback,
				  kAudioUnitScope_Global,
				  kOutputBus,
				  &callbackStruct,
				  sizeof(callbackStruct));
	checkStatus(status, "AudioUnitSetProperty SetInputCallback");

	UInt32 uFlag = 1;
	status = AudioUnitSetProperty(inAU,
			kAudioOutputUnitProperty_EnableIO,
			kAudioUnitScope_Input,
			kInputBus,
			&uFlag,
			sizeof(uFlag));
	checkStatus(status, "AudioUnitSetProperty EnableIO Scope_Input enable In");

	uFlag = 0;
	status = AudioUnitSetProperty(inAU,
			kAudioOutputUnitProperty_EnableIO,
			kAudioUnitScope_Output,
			kOutputBus,
			&uFlag,
			sizeof(uFlag));
	checkStatus(status, "AudioUnitSetProperty EnableIO Scope_Input disable Out");

	set_default_device(inAU);

	set_format(inAU, true);

	// Initialise
	status = AudioUnitInitialize(inAU);
	checkStatus(status, "AudioUnitInitialize in");
	fprintf(stdout, "Initialize InAudioUnit\n");

	status = AudioOutputUnitStart(inAU);
	checkStatus(status, "AudioUnitStart in");
	fprintf(stdout, "Start InAudioUnit\n");

	sleep(1);

	status = AudioOutputUnitStop(inAU);
	checkStatus(status, "AudioUnitStop in");
	fprintf(stdout, "Stop InAudioUnit\n");

	status = AudioUnitUninitialize(inAU);
	checkStatus(status, "AudioUnitUninitialize in");

	AudioComponentInstanceDispose(inAU);

	return 0;
}

