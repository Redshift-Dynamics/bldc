/*
	Copyright 2026

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
*/

#ifndef HW_REDSHIFT_TACHION_T18_H_
#define HW_REDSHIFT_TACHION_T18_H_

#define HW_NAME					"redshift_tachion_t18"
#define HW_REDSHIFT_TACHION_T18

#define REDSHIFT_TACHION_CURRENT_MEASUREMENT_LIMIT	550.0
#define MCCONF_L_MAX_ABS_CURRENT		550.0
#define HW_LIM_CURRENT			-550.0, 550.0
#define HW_LIM_CURRENT_IN		-250.0, 250.0
#define HW_LIM_CURRENT_ABS		0.0, 550.0

#include "hw_redshift_tachion_core.h"

#endif /* HW_REDSHIFT_TACHION_T18_H_ */
