#ifndef ORIENTATION_PROVIDER_H_FBK20220627
#define ORIENTATION_PROVIDER_H_FBK20220627

#include <vector>

class OrientationProvider
{
 public:
	 OrientationProvider();
	 ~OrientationProvider();
	 virtual bool receive() = 0;
	 std::vector<double> output;

};
#endif
