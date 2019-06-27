#ifndef RBDL_URDFREADER_H
#define RBDL_URDFREADER_H

#include <vector>

#include <any_rbdl/rbdl_config.h>

namespace RigidBodyDynamics {

struct Model;

namespace Addons {
  ANY_RBDL_DLLAPI bool URDFReadFromFile (const char* filename, Model* model, bool verbose = false);
  ANY_RBDL_DLLAPI bool URDFReadFromString (const char* model_xml_string, Model* model, bool verbose = false);

  /** These methods are an overloaded version of the standard URDF reader
	* interface. They additionally consider a sequence of names referring to
	* movable joints in the URDF representation and add the RBDL joints to
	* the RBDL model in the order proposed by this sequence. Further, these
	* overloads support the case of a root body without inertia, assuming that
	* there exists a descendant in the URDF model whose name corresponds to
	* the name of the root body suffixed by "_inertia" and whose list of child
	* nodes contains an inertial node.
	*/
  ANY_RBDL_DLLAPI bool URDFReadFromFile (const char* filename, Model* model,
									 const std::vector<std::string>& movable_joints_in_order,
									 bool verbose = false);
  ANY_RBDL_DLLAPI bool URDFReadFromString (const char* model_xml_string, Model* model,
									   const std::vector<std::string>& movable_joints_in_order,
									   bool verbose = false);
}

}

/* _RBDL_URDFREADER_H */
#endif
