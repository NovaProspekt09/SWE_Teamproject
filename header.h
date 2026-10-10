#ifndef CONTROLLER_H
#define CONTROLLER_H

/* Timing in logical ticks; adjust to the system sampling interval. */
#define RVC_TURN_TICKS 5
#define RVC_BACKWARD_TICKS 3
#define RVC_POWER_UP_TICKS 5

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== Data Types ===================== */
typedef enum {
    OBS_NONE = 0,
    OBS_FRONT,
    OBS_LEFT,
    OBS_RIGHT,
    OBS_FRONT_LEFT,
    OBS_FRONT_RIGHT,
    OBS_LEFT_RIGHT,
    OBS_ALL
} ObstacleLocation;

typedef enum { DUST_NONE = 0, DUST_NORMAL } DustLevel;

typedef enum {
    ACT_IDLE = 0,
    ACT_MOVE_FORWARD,
    ACT_TURN_LEFT,
    ACT_TURN_RIGHT,
    ACT_MOVE_BACKWARD
} DriveAction;

typedef enum { CLN_OFF = 0, CLN_NORMAL, CLN_POWER_UP } CleanAction;

typedef struct {
    DriveAction drive;
    CleanAction clean;
} RvcCommand;

/* ============ Library Modules (HW Interface) ============ */
bool Front_sensor_Interface(void);
bool Left_sensor_Interface(void);
bool Right_sensor_Interface(void);
int  Dust_sensor_Interface(void);          /* 0 ~ 100 */
void Motor_Interface(DriveAction a);
void Cleaner_Interface(CleanAction a);

/* ================== Afferent Modules =================== */
ObstacleLocation Determine_Obstacle_location(void);
DustLevel        Determine_dust_existence(void);

/* ================== Efferent Modules =================== */
/* Enable/Disable: only state transitions send commands. */
void Move_Forward_Module(bool enable);
void Move_Backward_Module(bool enable);
/* Trigger: start one non-blocking turn; further ticks advance it internally. */
void Turn_Left_Module(void);
void Turn_Right_Module(void);
void Cleaner_normal_Module(void);
void Dust_Clean_off_Module(void);
void Power_up_Clean_Module(void);

/* ============== Central Transformation ================= */
/* Sends OFF/IDLE and resets all state; also safe to call during operation. */
void       Controller_Init(void);
RvcCommand Controller(void);
unsigned int Controller_GetTick(void);
RvcCommand Controller_GetLast(void);
/* Implemented by the emulated interface, or by test doubles in RVC_TEST. */
CleanAction Cleaner_Interface_GetLast(void);
DriveAction Motor_Interface_GetLast(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROLLER_H */
