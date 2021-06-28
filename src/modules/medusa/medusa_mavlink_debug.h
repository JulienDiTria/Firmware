#pragma once

#define MD_STATUS "MD_STATUS" // x = log, y = current sample, z = sampling status
#define MD_STREAM "MD_STREAM" // x = depth in [cm], y = delta pressure in filter in [mbar], z = volume of water sampled in [ml]
#define MD_SAM_1 "MD_SAM_1" // x = numero of sample in [index nb], y = volume gathered through the sample in [ml], z = depth of the sample in [meter]
#define MD_SAM_2 "MD_SAM_2" // x = starting time of sample in [ms], y = end time of sample in [ms] since pod boot, z = expected time in [ms] for taking the sample
#define MD_SAM_3 "MD_SAM_3" // x = DP in tubes at start of sampling in [mbar], y = DP in tubes at end of sampling in [mbar], z = unused

