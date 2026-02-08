#pragma once

#include "seahowl/fluid/hydro/morison.h"
#include "seahowl/fluid/hydro/foundation_fluid.h"
#include "seahowl/fluid/hydro/floater_hydro.h"
#include "seahowl/fluid/hydro/monopile_hydro.h"
#include "seahowl/fluid/hydro/mooring_hydro.h"
#ifdef HAVE_HYDROCHRONO
    #include "seahowl/fluid/hydro/hydrochrono_adapter.h"
#endif
#ifdef HAVE_HYDRODYN
    #include "seahowl/fluid/hydro/hydrodyn_adapter.h"
#endif
