/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef INVERTER

#include "drivers/io.h"
#include "drivers/io_impl.h"

#include "inverter.h"

static ioRec_t *pin = NULL;

void initInverter(void)
{
    pin = IO_REC(INVERTER_IO);
    IOInit(pin, OWNER_SYSTEM, RESOURCE_OUTPUT);
    IOConfigGPIO(pin, IOCFG_OUT_PP);
}

void inverterSet(bool on)
{
    IOWrite(pin, on);
}

#endif
