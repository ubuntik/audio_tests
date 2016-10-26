// g++ -framework AudioUnit -framework AudioToolbox -framework CoreAudio -framework CoreFoundation -framework Foundation -I/usr/local/include -L/usr/local/lib -lspeex -lspeexdsp -lpthread -o au -Wall audio-speex.mm

#include <iostream>
#include <pthread.h>
#include <speex/speex.h>
#include <speex/speex_echo.h>
#include <speex/speex_preprocess.h>

#include <AudioUnit/AudioUnit.h>
#include <AudioToolbox/AudioToolbox.h>
#include <CoreAudio/CoreAudio.h>
#include <CoreFoundation/CoreFoundation.h>

#define MUTEX pthread_mutex_t
#define INIT(m) pthread_mutex_init(&m, NULL);
#define DESTROY(m) pthread_mutex_destroy(&m);
#define LOCK(m) pthread_mutex_lock(&m)
#define UNLOCK(m) pthread_mutex_unlock(&m)

#define max(a, b) (((a) > (b)) ? (a) : (b))
#define min(a, b) (((a) < (b)) ? (a) : (b))

#define kOutputBus 0
#define kInputBus 1

#define FS 4
#define SZ 512

struct el {
	void *next;
	void *data;
};

class ring_buf
{
public:
	ring_buf()
	{
		m_buf[0].data = calloc(SZ, 1);
		for(int i = 1; i < FS; i++) {
			m_buf[i].data = calloc(SZ, 1);
			m_buf[i - 1].next = m_buf[i].data;
		}
		m_buf[FS - 1].next = m_buf[0].data;
		m_crt = 0;
		INIT(m_mutex);
	};

	~ring_buf()
	{
		for(int i = 0; i < FS; i++)
			free(m_buf[i].data);
		DESTROY(m_mutex);
	};

	void get(void *src) { LOCK(m_mutex); shift(); memcpy(src, m_buf[m_crt].data, SZ); UNLOCK(m_mutex); };
	void set(void *src) { LOCK(m_mutex); memcpy(m_buf[m_crt].next , src, SZ); shift(); UNLOCK(m_mutex); };

private:
	void shift() { m_crt = (m_crt + 1) % FS; };

	volatile int m_crt;
	MUTEX m_mutex;
	el m_buf[FS];
};

static void checkStatus(OSStatus status, const char *func)
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

	std::cerr << "[" << func << "]: " << serr << " (" << (int)status << ")" << std::endl;
}

/*
	mic -> Input_Scope    / kInputBus /    Output_Scope -> app
	hph <- Output_Scope   / kOutputBus /   Input_Scope  <- app
*/

class MyRecorder {
public:
	MyRecorder();
	~MyRecorder();

	void start();
	void stop();
	AudioUnit get_au() { return m_au; };

	OSStatus set_input_callback();
	OSStatus set_default_device();
	OSStatus set_format();
	OSStatus set_IO_enable();

	FILE *fd;
	SpeexEchoState *estate;
	SpeexPreprocessState *pstate;
private:
	AudioUnit m_au;
};

int cnt = 0;

static OSStatus recordingCallback(void *inRefCon,
		  AudioUnitRenderActionFlags *ioActionFlags,
		  const AudioTimeStamp *inTimeStamp,
		  UInt32 inBusNumber,
		  UInt32 inNumberFrames,
		  AudioBufferList *ioData) {

	std::cout << ">>> recordingCallback: frames = " << inNumberFrames << " bus = " << inBusNumber << std::endl;

	AudioBufferList bufferList;
	size_t bytesPerSample = sizeof (Float32);
	size_t bytesPerFrame = inNumberFrames * bytesPerSample;
	bufferList.mNumberBuffers = 1;
	bufferList.mBuffers[0].mNumberChannels = 1;
	bufferList.mBuffers[0].mData = NULL;
	bufferList.mBuffers[0].mDataByteSize = bytesPerFrame;
	MyRecorder *rec = (MyRecorder *)inRefCon;

	OSStatus status;
	AudioUnit audioUnit = (AudioUnit)(rec->get_au());
	status = AudioUnitRender(audioUnit,
				ioActionFlags,
				inTimeStamp,
				inBusNumber,
				inNumberFrames,
				&bufferList);
	checkStatus(status, "AudioUnitRender");
	float *data = (float *)bufferList.mBuffers[0].mData;
	float *echoed = (float *)malloc(bytesPerFrame);
	short *in = (short *)malloc(inNumberFrames * sizeof(short));
	short *out = (short *)malloc(inNumberFrames * sizeof(short));

	for (int i = 0; i < inNumberFrames; i++)
		in[i] = (short)(data[i] * 32767.0f);

	short *decimated = (short *)calloc(inNumberFrames, bytesPerSample);
//	memcpy((void *)out, (void *)in, inNumberFrames * sizeof(short));
	speex_echo_cancellation(rec->estate, in, decimated, out);
	speex_preprocess_run(rec->pstate, out);

	for (int i = 0; i < inNumberFrames; i++)
		echoed[i] = (float)out[i] / 32767.0f;

	for (int i = 0; i < inNumberFrames; i++)
		fprintf(rec->fd, "%d %f %f\n", cnt++, data[i], echoed[i]);

	return noErr;
}

OSStatus MyRecorder::set_input_callback()
{
	OSStatus status;

	AURenderCallbackStruct callbackStruct;
	callbackStruct.inputProc = recordingCallback;
	callbackStruct.inputProcRefCon = (void *)this;
	status = AudioUnitSetProperty(m_au,
				  kAudioOutputUnitProperty_SetInputCallback,
				  kAudioUnitScope_Global,
				  kOutputBus,
				  &callbackStruct,
				  sizeof(callbackStruct));
	checkStatus(status, "AudioUnitSetProperty SetInputCallback");
	if (status != 0)
		return status;

	return noErr;
}

OSStatus MyRecorder::set_default_device()
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
	if (status != 0)
		return status;

	status = AudioUnitSetProperty(m_au,
			kAudioOutputUnitProperty_CurrentDevice,
			kAudioUnitScope_Global,
			kInputBus,
			&defaultDevice,
			sizeof(defaultDevice));
	checkStatus(status, "AudioUnitSetProperty CurrentDevice");
	if (status != 0)
		return status;

	return noErr;
}

OSStatus MyRecorder::set_format()
{
	OSStatus status;

	AudioStreamBasicDescription AudioFormat;
	UInt32 size = sizeof(AudioFormat);

	status = AudioUnitGetProperty(m_au,
			kAudioUnitProperty_StreamFormat,
			kAudioUnitScope_Input,
			kInputBus,
			&AudioFormat,
			&size);
	checkStatus(status, "AudioUnitGetProperty StreamFormat");
	if (status != 0)
		return status;

	std::cout << "AudioFormat: " << std::endl;
	std::cout << "format_id=" << AudioFormat.mFormatID << std::endl;
	std::cout << "format_flags=" << AudioFormat.mFormatFlags << std::endl;
	std::cout << "frames_per_pack=" << AudioFormat.mFramesPerPacket << std::endl;
	std::cout << "channels_per_frame=" << AudioFormat.mChannelsPerFrame << std::endl;
	std::cout << "bits_per_channel=" <<  AudioFormat.mBitsPerChannel << std::endl;
	std::cout << "sample_rate=" << AudioFormat.mSampleRate <<std::endl;

	AudioFormat.mReserved		= 0;
	AudioFormat.mFormatID		= kAudioFormatLinearPCM;
	AudioFormat.mFormatFlags	= kAudioFormatFlagIsFloat | kAudioFormatFlagsNativeEndian | kAudioFormatFlagIsPacked;
	AudioFormat.mBitsPerChannel	= 32;
	AudioFormat.mFramesPerPacket	= 1;
	AudioFormat.mBytesPerFrame	= AudioFormat.mBitsPerChannel * AudioFormat.mChannelsPerFrame / 8;
	AudioFormat.mBytesPerPacket	= AudioFormat.mBytesPerFrame * AudioFormat.mFramesPerPacket;

	status = AudioUnitSetProperty(m_au,
			kAudioUnitProperty_StreamFormat,
			kAudioUnitScope_Output,
			kInputBus,
			&AudioFormat,
			sizeof(AudioFormat));
	checkStatus(status, "AudioUnitSetProperty StreamFormat Scope_In/Output");
	if (status != 0)
		return status;

	return noErr;
}

OSStatus MyRecorder::set_IO_enable()
{
	OSStatus status;

	UInt32 uFlag = 1;
	status = AudioUnitSetProperty(m_au,
			kAudioOutputUnitProperty_EnableIO,
			kAudioUnitScope_Input,
			kInputBus,
			&uFlag,
			sizeof(uFlag));
	checkStatus(status, "AudioUnitSetProperty EnableIO Scope_Input enable In");
	if (status != 0)
		return status;

	uFlag = 0;
	status = AudioUnitSetProperty(m_au,
			kAudioOutputUnitProperty_EnableIO,
			kAudioUnitScope_Output,
			kOutputBus,
			&uFlag,
			sizeof(uFlag));
	checkStatus(status, "AudioUnitSetProperty EnableIO Scope_Input disable Out");
	if (status != 0)
		return status;

	return noErr;
}

void MyRecorder::start()
{
	OSStatus status;

	status = AudioUnitInitialize(m_au);
	checkStatus(status, "AudioUnitInitialize");
	std::cout << "Initialize InAudioUnit" << std::endl;

	status = AudioOutputUnitStart(m_au);
	checkStatus(status, "AudioUnitStart");
	std::cout << "Start InAudioUnit" << std::endl;
}

void MyRecorder::stop()
{
	OSStatus status;

	status = AudioOutputUnitStop(m_au);
	checkStatus(status, "AudioUnitStop");
	std::cout << "Stop InAudioUnit\n" << std::endl;

	status = AudioUnitUninitialize(m_au);
	checkStatus(status, "AudioUnitUninitialize");
}

MyRecorder::MyRecorder()
{
	OSStatus status;

	AudioComponentDescription desc;
	desc.componentType = kAudioUnitType_Output;
	desc.componentSubType = kAudioUnitSubType_HALOutput;
	desc.componentManufacturer = kAudioUnitManufacturer_Apple;
	desc.componentFlags = 0;
	desc.componentFlagsMask = 0;

	AudioComponent component = AudioComponentFindNext(NULL, &desc);

	status = AudioComponentInstanceNew(component, &m_au);
	checkStatus(status, "AudioComponentInstanceNew");

	fd = fopen("plot.dat", "w");
	if (fd == NULL)
		std::cerr << "Cannot open file plot.dat" << std::endl;

	UInt32 sampleRate = 44100;
	estate = speex_echo_state_init(294, 14700);
	pstate = speex_preprocess_state_init(294, sampleRate);
	speex_echo_ctl(estate, SPEEX_ECHO_SET_SAMPLING_RATE, &sampleRate);
	speex_preprocess_ctl(pstate, SPEEX_PREPROCESS_SET_ECHO_STATE, estate);
}

MyRecorder::~MyRecorder()
{
	AudioComponentInstanceDispose(m_au);
	speex_echo_state_destroy(estate);
	speex_preprocess_state_destroy(pstate);
	fclose(fd);
}

int main(int argc, char **argv)
{
	OSStatus status;

	MyRecorder recorder;

	status = recorder.set_input_callback();
	if (status != 0) {
		std::cerr << "Cannot set input callback" << std::endl;
		return -1;
	}

	status = recorder.set_IO_enable();
	if (status != 0) {
		std::cerr << "Cannot set IO enable" << std::endl;
		return -1;
	}

	status = recorder.set_default_device();
	if (status != 0) {
		std::cerr << "Cannot set default device" << std::endl;
		return -1;
	}

	status = recorder.set_format();
	if (status != 0) {
		std::cerr << "Cannot set format" << std::endl;
		return -1;
	}

	recorder.start();

	std::string f;
	std::getline(std::cin, f);

	recorder.stop();

	return 0;
}

