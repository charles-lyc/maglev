/**
 * Shim main.h that redirects to our HAL mock for simulation.
 * This replaces cube/Core/Inc/main.h when compiling on the host.
 */
#pragma once
#include "hal_mock.h"
