/*
 * mlx90614.h
 *
 *  Created on: Nov 13, 2025
 *      Author: aaron
 */

#ifndef MLX90614_H_
#define MLX90614_H_

#include <stdint.h>

#define MLX90614_ADDR 0x5A

int mlx90614_read_ambient(int32_t *t_mdegC);
int mlx90614_read_object(int32_t *t_mdegC);

#endif /* MLX90614_H_ */
