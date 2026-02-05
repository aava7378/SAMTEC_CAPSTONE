/*
 * max1968_shdn.h
 *
 *  Created on: Jan 14, 2026
 *      Author: aaron
 */

#ifndef MAX1968_SHDN_H_
#define MAX1968_SHDN_H_

#include <stdint.h>

void max1968_shdn_init(void);
void max1968_shdn_set(uint8_t en);   // 0=shutdown, 1=enable

#endif /* MAX1968_SHDN_H_ */
