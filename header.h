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

typedef enum { DUST_NONE = 0, DUST_NORMAL, DUST_HEAVY } DustLevel;

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
bool Front_Sensor_Interface(void);
bool Left_Sensor_Interface(void);
bool Right_Sensor_Interface(void);
int  Dust_Sensor_Interface(void);          /* 0 ~ 100 */
void Motor_Interface(DriveAction a);
void Cleaner_Interface(CleanAction a);

/* ================== Afferent Modules =================== */
ObstacleLocation Determine_Obstacle_Location(bool f, bool l, bool r);
DustLevel        Determine_Dust_Existence(int dust_raw);
bool             Dust_Existence(DustLevel lv);

/* ================== Efferent Modules =================== */
void Move_Forward_Module(void);
void Move_Backward_Module(void);
void Turn_Left_Module(void);
void Turn_Right_Module(void);
void Cleaner_Normal_Module(void);
void Dust_Clean_Off_Module(void);
void Power_Up_Clean_Module(void);

/* ============== Central Transformation ================= */
void       Controller_Init(void);
RvcCommand Controller_Step(ObstacleLocation obs, DustLevel dust);
RvcCommand Controller_GetLast(void);
void       Controller_PowerUpSequence(void);

/* ===================== Main Loop ======================= */
void Rvc_Tick(void);
int  Rvc_Main(int max_tick);   /* max_tick < 0 이면 무한 루프 */

#ifdef __cplusplus
}
#endif

#endif /* CONTROLLER_H */