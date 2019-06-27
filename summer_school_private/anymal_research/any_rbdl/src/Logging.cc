/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2015 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include "any_rbdl/Logging.h"

ANY_RBDL_DLLAPI std::ostringstream LogOutput;

ANY_RBDL_DLLAPI
void ClearLogOutput() {
	LogOutput.str("");
}
