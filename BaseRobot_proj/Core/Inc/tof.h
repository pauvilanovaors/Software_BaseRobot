/*
 * tof.h
 *
 *  Created on: Aug 15, 2022
 *      Author: pauvi
 */

#ifndef INC_TOF_H_
#define INC_TOF_H_

// Read the model and revision of the
// tof sensor
//


int tofGetModel(int *model, int *revision, uint8_t devAddr);

int tofReadDistance(uint8_t devAddr);

int tofInit(int bLongRange, uint8_t devAddr);

void SetDevAddr(uint8_t new_addr,uint8_t last_addr);


#endif /* INC_TOF_H_ */
