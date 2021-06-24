#pragma once

#define MEDUSA_DEBUG_INDEX (470872.67970313) // index for first data of debug msg, to filter out other debug msg

#define MEDUSA_DEBUG_TIMESTAMP (0.0) // timestamp in [ms] since pod boot for next data from 1.0 to 2.5
#define MEDUSA_DEBUG_LOG (0.5) // flag for logging in sd card [0 for not logging, 1 for logging]
#define MEDUSA_DEBUG_CUR_SMPL_NBR (0.750) // current sample number being streamed [0 or 1]

#define MEDUSA_DEBUG_DEPTH (1.0) // depth in [meter]
#define MEDUSA_DEBUG_PITCH (1.5) // pitch angle in [degree]
#define MEDUSA_DEBUG_DELTP (2.0) // delta pressure in filter in [mbar]
#define MEDUSA_DEBUG_VOLUM (2.5) // volume of water sampled in [ml]

#define MEDUSA_DEBUG_SMPL_NUMBR (3.000) // numero of sample in [index nb]
#define MEDUSA_DEBUG_SMPL_VOLUM (3.125) // volume gathered through the sample in [ml]
#define MEDUSA_DEBUG_SMPL_DEPTH (3.250) // depth of the sample in [meter]
#define MEDUSA_DEBUG_SMPL_TIME_START (3.375) // starting time of sample in [ms] since pod boot
#define MEDUSA_DEBUG_SMPL_TIME_END (3.500) // end time of sample in [ms] since pod boot
#define MEDUSA_DEBUG_SMPL_TIME_NEEDED (3.756250) // expected time in [ms] for taking the sample
#define MEDUSA_DEBUG_SMPL_DP_START (3.750) // DP in tubes at start of sampling in [mbar]
#define MEDUSA_DEBUG_SMPL_DP_END (3.875) // DP in tubes at end of sampling in [mbar]

