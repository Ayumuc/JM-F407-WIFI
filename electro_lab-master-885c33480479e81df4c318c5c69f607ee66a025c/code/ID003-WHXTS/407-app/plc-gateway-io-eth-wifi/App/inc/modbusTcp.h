/*
 * modbusTcp.h
 *
 *  Created on: Sep 28, 2021
 *      Author: Administrator
 */

#ifndef INC_MODBUSTCP_H_
#define INC_MODBUSTCP_H_

uint8_t* parseModbusCommand(uint8_t *modbusCmd, int32_t comLength, uint8_t* size);

uint32_t writedata_to_flash(uint8_t *data, uint32_t len, uint32_t address);
#endif /* INC_MODBUSTCP_H_ */
