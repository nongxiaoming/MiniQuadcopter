#ifndef __DRV_MOTORS_H
#define __DRV_MOTORS_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
#define MOTORS_NUM_MAX    4	 
#define MOTORS_PWM_MAX 2000
#define MOTORS_PWM_MIN 1000	 
	 
void rt_motors_hw_init(void);	 
	 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
