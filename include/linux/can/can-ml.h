/*
 * Copyright (C) 2017 Pengutronix, Marc Kleine-Budde <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef CAN_ML_H
#define CAN_ML_H

#include "../../net/can/af_can.h"

struct can_ml_priv {
	struct can_dev_rcv_lists dev_rcv_lists;
};

#endif /* CAN_ML_H */
