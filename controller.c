/* ============================================================
 *  RVC (Robot Vacuum Cleaner) - Structured Design Implementation
 *  Team Project #2 / Software Engineering
 *  Build (run)  : gcc -std=c17 -o rvc controller.c
 *  Build (test) : gcc -std=c17 -DRVC_TEST controller.c version2_system_test.c -o rvc_test
 * ============================================================ */
#include <stdio.h>
#include "header.h"

/* ------------------ Internal State ------------------ */
static RvcCommand g_last;
static DriveAction g_drive;
static int g_motion_ticks;
static int g_power_up_ticks;
static unsigned int g_tick;
#ifndef RVC_TEST
static CleanAction g_interface_clean = CLN_OFF;
#endif

void Controller_Init(void) {
    g_tick = 0;
#ifndef RVC_TEST
    g_interface_clean = CLN_OFF;
#endif
    g_last.drive = ACT_IDLE;
    g_last.clean = CLN_OFF;
    g_drive = ACT_MOVE_FORWARD;
    g_motion_ticks = 0;
    g_power_up_ticks = 0;
}

/* ============== Afferent Flow (Input) =============== */
ObstacleLocation Determine_Obstacle_Location(void) {
    bool f = Front_Sensor_Interface();
    bool l = Left_Sensor_Interface();
    bool r = Right_Sensor_Interface();
    if (f && l && r) return OBS_ALL;
    if (f && l)      return OBS_FRONT_LEFT;
    if (f && r)      return OBS_FRONT_RIGHT;
    if (l && r)      return OBS_LEFT_RIGHT;
    if (f)           return OBS_FRONT;
    if (l)           return OBS_LEFT;
    if (r)           return OBS_RIGHT;
    return OBS_NONE;
}

DustLevel Determine_Dust_Existence(void) {
    int dust_raw = Dust_Sensor_Interface();
    if (dust_raw >= 20) return DUST_NORMAL;
    return DUST_NONE;
}


/* ============== Efferent Flow (Output) ============== */
void Move_Forward_Module(void)   { Motor_Interface(ACT_MOVE_FORWARD);  }
void Move_Backward_Module(void)  { Motor_Interface(ACT_MOVE_BACKWARD); }
void Turn_Left_Module(void)      { Motor_Interface(ACT_TURN_LEFT);     }
void Turn_Right_Module(void)     { Motor_Interface(ACT_TURN_RIGHT);    }
void Cleaner_Normal_Module(void) { Cleaner_Interface(CLN_NORMAL);      }
void Dust_Clean_Off_Module(void) { Cleaner_Interface(CLN_OFF);         }
void Power_Up_Clean_Module(void) { Cleaner_Interface(CLN_POWER_UP);    }

/* =========== Central Transformation (Control) ======== */
/* Left has priority when both sides are available. After backing up,
 * rotate left if both sides remain blocked, rather than entering STOP.
 * This is an emulated escape policy, not a guarantee of physical clearance. */
static DriveAction ChooseTurn(ObstacleLocation obs) {
    bool left_blocked = obs == OBS_LEFT || obs == OBS_FRONT_LEFT ||
                        obs == OBS_LEFT_RIGHT || obs == OBS_ALL;
    bool right_blocked = obs == OBS_RIGHT || obs == OBS_FRONT_RIGHT ||
                         obs == OBS_LEFT_RIGHT || obs == OBS_ALL;
    return !left_blocked || right_blocked ? ACT_TURN_LEFT : ACT_TURN_RIGHT;
}

RvcCommand Controller(void) {
    ObstacleLocation obs = Determine_Obstacle_Location();
    DustLevel dust = Determine_Dust_Existence();
    RvcCommand cmd;
    cmd.clean = g_last.clean;

    /* A command persists for its specified number of logical ticks. */
    if (g_drive != ACT_MOVE_FORWARD) {
        if (g_motion_ticks > 0) --g_motion_ticks;
        if (g_motion_ticks == 0) {
            if (g_drive == ACT_MOVE_BACKWARD) {
                g_drive = ChooseTurn(obs);
                g_motion_ticks = RVC_TURN_TICKS;
            } else {
                g_drive = ACT_MOVE_FORWARD;
            }
        }
    }

    /* Side obstacles alone do not prevent forward movement. */
    if (g_drive == ACT_MOVE_FORWARD) {
        switch (obs) {
        case OBS_ALL:
            g_drive = ACT_MOVE_BACKWARD;
            g_motion_ticks = RVC_BACKWARD_TICKS;
            break;
        case OBS_FRONT:
        case OBS_FRONT_LEFT:
        case OBS_FRONT_RIGHT:
            g_drive = ChooseTurn(obs);
            g_motion_ticks = RVC_TURN_TICKS;
            break;
        default:
            break;
        }
    }
    cmd.drive = g_drive;

    /* Decide movement first, but stop cleaning before sending avoidance motion.
     * The start call is tick 1; four following calls retain boost without commands.
     * On call 6 the expired timer permits a fresh dust decision. */
    if (cmd.drive != ACT_MOVE_FORWARD) {
        g_power_up_ticks = 0;
        if (cmd.clean != CLN_OFF) {
            Dust_Clean_Off_Module();
            cmd.clean = CLN_OFF;
        }
    } else if (g_power_up_ticks > 0) {
        --g_power_up_ticks;
    } else if (dust == DUST_NORMAL) {
        Power_Up_Clean_Module();
        cmd.clean = CLN_POWER_UP;
        g_power_up_ticks = RVC_POWER_UP_TICKS - 1;
    } else if (cmd.clean != CLN_NORMAL) {
        Cleaner_Normal_Module();
        cmd.clean = CLN_NORMAL;
    }

    switch (cmd.drive) {
    case ACT_MOVE_FORWARD: Move_Forward_Module(); break;
    case ACT_MOVE_BACKWARD: Move_Backward_Module(); break;
    case ACT_TURN_LEFT: Turn_Left_Module(); break;
    case ACT_TURN_RIGHT: Turn_Right_Module(); break;
    default: break;
    }
    g_last = cmd;
    ++g_tick; /* One logical cycle, not elapsed wall-clock time. */
    return cmd;
}

unsigned int Controller_GetTick(void) { return g_tick; }
RvcCommand Controller_GetLast(void) { return g_last; }

/* Test builds supply hardware doubles and their own main(). */
#ifndef RVC_TEST

/* ---- Library Modules: HW Emulation ---- */


bool Front_Sensor_Interface(void) { return (g_tick % 7) == 3; }
bool Left_Sensor_Interface(void)  { return (g_tick % 11) == 5; }
bool Right_Sensor_Interface(void) { return (g_tick % 13) == 8; }
int  Dust_Sensor_Interface(void)  { return (int)(((g_tick % 100u) * 17u) % 100u); }

void Motor_Interface(DriveAction a) {
    static const char* n[] = {"IDLE","FORWARD","TURN_LEFT","TURN_RIGHT","BACKWARD"};
    printf("[MOTOR  ] %s\n", n[a]);
}
void Cleaner_Interface(CleanAction a) {
    static const char* n[] = {"OFF","NORMAL","POWER_UP"};
    g_interface_clean = a;
    printf("[CLEANER] %s\n", n[a]);
}

CleanAction Cleaner_Interface_GetLast(void) { return g_interface_clean; }

int main(void) {
    int i;
    printf("=== RVC Start (logical ticks; no real-time wait) ===\n");
    Controller_Init();
    for (i = 0; i < 20; ++i) {
        RvcCommand cmd = Controller();
        printf("[STATE  ] tick=%u drive=%d clean=%d interface=%d\n",
            Controller_GetTick() - 1u, (int)cmd.drive, (int)cmd.clean,
            (int)Cleaner_Interface_GetLast());
    }
    printf("=== RVC End ===\n");
    return 0;
}

#endif /* RVC_TEST */
