#ifndef PIPELINE_AGRFM_HEADER_FBK_31052022
#define PIPELINE_AGRFM_HEADER_FBK_31052022

#include "experimental/AccelerationBasedPhaseDetector.h"
#include "osrt_ros/Pipeline/grf_pipe.h"

namespace Pipeline
{
	class Acc: public Pipeline::Grf
	{
		//OpenSimRT::AccelerationBasedPhaseDetector* detector;
		const std::string section = "TEST_ACCELERATION_GRFM_PREDICTION_FROM_FILE";	
		public:
			Acc();
			~Acc();
	};
}
#endif
