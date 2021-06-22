/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file medusa_params.c
 * Parameters for medusa drone.
 *
 * @author Julien Di Tria 
 * @author Andre Farinha 
 */

/**
 * Medusa depth target
 *
 * Depth target to control the pod depth in [m]
 *
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group Medusa
 */
PARAM_DEFINE_FLOAT(MDSA_DEPTH_TRGT, 0.0f);

/**
 * Medusa sample status
 *
 * Status of the water sample : 0 - not sampling, 1 sampling on filter 1 , 2 sampling on filter 2. Changed back to 0 by underwater pod. 
 *
 * @min 0
 * @max 2
 * @decimal 0
 * @increment 1
 * @group Medusa
 */
PARAM_DEFINE_FLOAT(MDSA_SMPL_STATUS, 0);

/**
 * Medusa sample depth
 *
 * Depth at which the sample is being done in [m]
 *
 * @min 0
 * @max 10
 * @decimal 3
 * @increment 0.01
 * @group Medusa
 */
PARAM_DEFINE_FLOAT(MDSA_SMPL_DEPTH, 0.0f);

/**
 * Medusa sample volume
 *
 * Volumne of water currently sampled in [ml]
 *
 * @min 0
 * @max 1000
 * @decimal 3
 * @increment 0.01
 * @group Medusa
 */
PARAM_DEFINE_FLOAT(MDSA_SMPL_VOL, 0.0f);

/**
 * Medusa sample differential pressure
 *
 * Differential pressure between the starting of the sample and now, in [mbar]
 *
 * @min 0
 * @max 1000
 * @decimal 3
 * @increment 0.01
 * @group Medusa
 */
PARAM_DEFINE_FLOAT(MDSA_SMPL_DP, 0.0f);

