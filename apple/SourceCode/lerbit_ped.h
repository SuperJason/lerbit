#ifndef __LERBIT_PED_H
#define __LERBIT_PED_H

#include <stdint.h>

typedef int16_t lerbit_ped_acc_data_t;

extern uint32_t lerbit_ped_get_steps(void);
extern uint32_t lerbit_ped_get_distance(void);
extern void lerbit_ped_monitor(lerbit_ped_acc_data_t ped_data_X, lerbit_ped_acc_data_t ped_data_Y);
extern void lerbit_ped_init(void);

#endif /* __LERBIT_PED_H */

