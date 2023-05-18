/**\
 * Copyright (c) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#ifndef _COMMON_H
#define _COMMON_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "bmi2.h"

void bmi2_error_codes_print_result(int8_t rslt);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* _COMMON_H */
