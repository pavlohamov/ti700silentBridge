/*
 * bq24296.hpp
 *
 *  Created on: 3 Dec 2025
 *      Author: pavloha
 */

#pragma once


typedef struct i2c_master_dev_t *i2c_master_dev_handle_t;

namespace TI {

class BQ24296 {
public:
	BQ24296(i2c_master_dev_handle_t dev);

	void dump();

	int read(uint8_t addr);

private:
	const i2c_master_dev_handle_t _dev;

	int read(uint8_t *regs, size_t len);
};
}
