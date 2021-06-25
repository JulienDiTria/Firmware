#pragma once

#define MEDUSA_DEBUG_INDEX (9604778) // index for first data of debug msg, to filter out other debug msg

#define MEDUSA_DEBUG_TIMESTAMP (0) // timestamp in [ms] since pod boot for next data from 1.0 to 2.5
#define MEDUSA_DEBUG_LOG (1) // flag for logging in sd card [0 for not logging, 1 for logging]
#define MEDUSA_DEBUG_CUR_SMPL_NBR (2) // current sample number being streamed [0 or 1]

#define MEDUSA_DEBUG_DEPTH (10) // depth in [meter]
#define MEDUSA_DEBUG_PITCH (11) // pitch angle in [degree]
#define MEDUSA_DEBUG_DELTP (12) // delta pressure in filter in [mbar]
#define MEDUSA_DEBUG_VOLUM (13) // volume of water sampled in [ml]

#define MEDUSA_DEBUG_SMPL_NUMBR (20) // numero of sample in [index nb]
#define MEDUSA_DEBUG_SMPL_VOLUM (21) // volume gathered through the sample in [ml]
#define MEDUSA_DEBUG_SMPL_DEPTH (22) // depth of the sample in [meter]
#define MEDUSA_DEBUG_SMPL_TIME_START (23) // starting time of sample in [ms] since pod boot
#define MEDUSA_DEBUG_SMPL_TIME_END (24) // end time of sample in [ms] since pod boot
#define MEDUSA_DEBUG_SMPL_TIME_NEEDED (25) // expected time in [ms] for taking the sample
#define MEDUSA_DEBUG_SMPL_DP_START (26) // DP in tubes at start of sampling in [mbar]
#define MEDUSA_DEBUG_SMPL_DP_END (27) // DP in tubes at end of sampling in [mbar]

