#ifndef PIPELINE_CGRFM_HEADER_FBK_01062022
#define PIPELINE_CGRFM_HEADER_FBK_01062022

#include "experimental/ContactForceBasedPhaseDetector.h"
#include "Pipeline/include/grf_pipe.h"

namespace Pipeline
{
	class Fc: public Pipeline::Grf
	{
		//OpenSimRT::ContactForceBasedPhaseDetector* detector;
		const std::string section = "TEST_CONTACT_FORCE_GRFM_PREDICTION_FROM_FILE";
		public:
			Fc();
			~Fc();
	};
}
#endif
	
