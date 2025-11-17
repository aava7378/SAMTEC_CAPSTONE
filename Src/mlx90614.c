/*
 * mlx90614.c
 *
 *  Created on: Nov 13, 2025
 *      Author: aaron
 */

#include "mlx90614.h"
#include "i2c.h"

#define REG_TA     0x06
#define REG_TOBJ1  0x07

static int convert_raw(uint16_t raw, int32_t *out)
{
    int32_t r = (int32_t)raw;

    int32_t mK = r * 20;
    int32_t mC = mK - 273150;

    *out = mC;
    return 0;
}

int mlx90614_read_ambient(int32_t *t_mdegC)
{
    uint16_t raw;
    int rc = i2c1_read_word(MLX90614_ADDR, REG_TA, &raw);
    if (rc < 0) {
        return rc;
    }
    return convert_raw(raw, t_mdegC);
}

int mlx90614_read_object(int32_t *t_mdegC)
{
    uint16_t raw;
    int rc = i2c1_read_word(MLX90614_ADDR, REG_TOBJ1, &raw);
    if (rc < 0) {
        return rc;
    }
    return convert_raw(raw, t_mdegC);
}
