#ifndef __DRV_MOTORS_H
#define __DRV_MOTORS_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
#define MOTORS_NUM_MAX    4	 
#define MOTORS_PWM_MAX 2000
#define MOTORS_PWM_MIN 1000	 
	 
void motors_hw_init(void);	 
void motors_set_pwm(uint16_t *value);
	 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

