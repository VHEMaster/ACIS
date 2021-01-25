#include "main.h"

extern void csps_loop(void);
extern void csps_init(void);
extern void csps_exti(void);
extern float csps_getangle14(void);
extern float csps_getangle23from14(float angle);
extern float csps_getangle23(void);
extern float csps_getrpm(void);
extern float csps_getuspa(void);
extern uint8_t csps_isrotates(void);
extern uint8_t csps_isfound(void);
extern uint8_t csps_iserror(void);

extern float csps_correctangle(float angle);
