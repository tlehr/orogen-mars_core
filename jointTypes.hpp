#ifndef JOINT_TYPES_HH
#define JOINT_TYPES_HH

#include <string>

namespace simulation
{

struct ParallelKinematic{
	std::string externalName;
	std::string internalName1;
	std::string internalName2;
};


}
#endif
