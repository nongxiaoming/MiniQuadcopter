/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * version.tmpl - version of the build
 */
#include <rtthread.h>

#include "config.h"
#include "param.h"

const char * V_SLOCAL_REVISION="1.0";
const char * V_SREVISION="1.0";
const char * V_STAG="1.0";
const char * V_BRANCH="1.0";
const char * V_PROFILE=P_NAME;
const rt_bool_t V_MODIFIED=RT_TRUE;

/* Version recoverable from the ground */
//const uint32_t V_REVISION_0={irevision0};
//const uint16_t V_REVISION_1={irevision1};

PARAM_GROUP_START(firmware)
//PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, revision0, &V_REVISION_0)
//PARAM_ADD(PARAM_UINT16 | PARAM_RONLY, revision1, &V_REVISION_1)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, modified, &V_MODIFIED)
PARAM_GROUP_STOP(firmware)

