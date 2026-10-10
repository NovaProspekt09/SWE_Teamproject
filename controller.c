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
static int g_backward_ticks;
static bool g_forward_enabled;
static bool g_backward_enabled;
/* Turn modules own their duration. Controller only advances/queries completion. */
static int g_turn_ticks;
static int g_power_up_ticks;
static unsigned int g_tick;
#ifndef RVC_TEST
static CleanAction g_interface_clean = CLN_OFF;
static DriveAction g_interface_motor = ACT_IDLE;
#endif

void Controller_Init(void) {
    /* Send real reset commands, including when interfaces are supplied by tests. */
    Dust_Clean_off_Module();
    Motor_Interface(ACT_IDLE);
    g_tick = 0;
    g_last.drive = ACT_IDLE;
    g_last.clean = CLN_OFF;
    g_drive = ACT_MOVE_FORWARD;
    g_backward_ticks = 0;
    g_turn_ticks = 0;
    g_forward_enabled = false;
    g_backward_enabled = false;
    g_power_up_ticks = 0;
}

/* ============== Afferent Flow (Input) =============== */
ObstacleLocation Determine_Obstacle_location(void) {
    bool f = Front_sensor_Interface();
    bool l = Left_sensor_Interface();
    bool r = Right_sensor_Interface();
    if (f && l && r) return OBS_ALL;
    if (f && l)      return OBS_FRONT_LEFT;
    if (f && r)      return OBS_FRONT_RIGHT;
    if (l && r)      return OBS_LEFT_RIGHT;
    if (f)           return OBS_FRONT;
    if (l)           return OBS_LEFT;
    if (r)           return OBS_RIGHT;
    return OBS_NONE;
}

DustLevel Determine_dust_existence(void) {
    int dust_raw = Dust_sensor_Interface();
    if (dust_raw >= 20) return DUST_NORMAL;
    return DUST_NONE;
}


/* ============== Efferent Flow (Output) ============== */
void Move_Forward_Module(bool enable) {
    if (enable == g_forward_enabled) return;
    g_forward_enabled = enable;
    Motor_Interface(enable ? ACT_MOVE_FORWARD : ACT_IDLE);
}
void Move_Backward_Module(bool enable) {
    if (enable == g_backward_enabled) return;
    g_backward_enabled = enable;
    Motor_Interface(enable ? ACT_MOVE_BACKWARD : ACT_IDLE);
}
void Turn_Left_Module(void) {
    g_turn_ticks = RVC_TURN_TICKS;
    Motor_Interface(ACT_TURN_LEFT);
}
void Turn_Right_Module(void) {
    g_turn_ticks = RVC_TURN_TICKS;
    Motor_Interface(ACT_TURN_RIGHT);
}
/* Called on ticks following the trigger, never waits or reads sensors. */
static bool AdvanceTurn(void) {
    if (g_turn_ticks > 0) --g_turn_ticks;
    return g_turn_ticks == 0;
}
void Cleaner_normal_Module(void) { Cleaner_Interface(CLN_NORMAL);      }
void Dust_Clean_off_Module(void) { Cleaner_Interface(CLN_OFF);         }
void Power_up_Clean_Module(void) { Cleaner_Interface(CLN_POWER_UP);    }

/* =========== Central Transformation (Control) ======== */
/* Left has priority when both sides are available. If neither side is open,
 * retain backward Enable until a later tick provides an escape direction. */
static DriveAction ChooseTurn(ObstacleLocation obs) {
    bool left_blocked = obs == OBS_LEFT || obs == OBS_FRONT_LEFT ||
                        obs == OBS_LEFT_RIGHT || obs == OBS_ALL;
    bool right_blocked = obs == OBS_RIGHT || obs == OBS_FRONT_RIGHT ||
                         obs == OBS_LEFT_RIGHT || obs == OBS_ALL;
    if (!left_blocked) return ACT_TURN_LEFT;
    if (!right_blocked) return ACT_TURN_RIGHT;
    return ACT_MOVE_BACKWARD;
}

RvcCommand Controller(void) {
    ObstacleLocation obs = Determine_Obstacle_location();
    DustLevel dust = Determine_dust_existence();
    RvcCommand cmd;
    cmd.clean = g_last.clean;

    /* Sensor sampling continues during a turn; completion does not block main. */
    if (g_drive == ACT_TURN_LEFT || g_drive == ACT_TURN_RIGHT) {
        if (AdvanceTurn()) g_drive = ACT_MOVE_FORWARD;
    } else if (g_drive == ACT_MOVE_BACKWARD) {
        if (g_backward_ticks > 0) --g_backward_ticks;
        /* After the initial 3 ticks, check every tick. Both sides blocked:
         * keep moving backward without re-enabling or resetting the timer. */
        if (g_backward_ticks == 0) g_drive = ChooseTurn(obs);
    }

    /* Side obstacles alone do not prevent forward movement. */
    if (g_drive == ACT_MOVE_FORWARD) {
        switch (obs) {
        case OBS_ALL:
            g_drive = ACT_MOVE_BACKWARD;
            g_backward_ticks = RVC_BACKWARD_TICKS;
            break;
        case OBS_FRONT:
        case OBS_FRONT_LEFT:
        case OBS_FRONT_RIGHT:
            g_drive = ChooseTurn(obs);
            break;
        default:
            break;
        }
    }
    cmd.drive = g_drive;

    /* Enter avoidance with OFF first; resume cleaning only after FORWARD. */
    if (cmd.drive != ACT_MOVE_FORWARD) {
        g_power_up_ticks = 0;
        if (cmd.clean != CLN_OFF) {
            Dust_Clean_off_Module();
            cmd.clean = CLN_OFF;
        }
    }

    /* Explicit Disable before a different movement is enabled/triggered.
     * Cleaner OFF above is always sent before avoidance motor commands. */
    if (g_forward_enabled && cmd.drive != ACT_MOVE_FORWARD)
        Move_Forward_Module(false);
    if (g_backward_enabled && cmd.drive != ACT_MOVE_BACKWARD)
        Move_Backward_Module(false);
    switch (cmd.drive) {
    case ACT_MOVE_FORWARD:
        if (!g_forward_enabled) Move_Forward_Module(true);
        break;
    case ACT_MOVE_BACKWARD:
        if (!g_backward_enabled) Move_Backward_Module(true);
        break;
    case ACT_TURN_LEFT:
        if (g_turn_ticks == 0) Turn_Left_Module();
        break;
    case ACT_TURN_RIGHT:
        if (g_turn_ticks == 0) Turn_Right_Module();
        break;
    default: break;
    }

    /* The start cycle counts as tick 1. Re-evaluate dust on cycle 6.
     * Motor FORWARD has already been sent before any cleaner restart here. */
    if (cmd.drive == ACT_MOVE_FORWARD) {
        if (g_power_up_ticks > 0) {
            --g_power_up_ticks;
        } else if (dust == DUST_NORMAL) {
            Power_up_Clean_Module();
            cmd.clean = CLN_POWER_UP;
            g_power_up_ticks = RVC_POWER_UP_TICKS - 1;
        } else if (cmd.clean != CLN_NORMAL) {
            Cleaner_normal_Module();
            cmd.clean = CLN_NORMAL;
        }
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


typedef struct { bool front, left, right; int dust; } DemoInput;
/* Scripted scenarios: normal -> full boost -> left turn -> right turn ->
 * backward escape -> boost interrupted by avoidance -> fresh recovery.
 * These are inputs only; movement and cleaning are still chosen by Controller. */
static const DemoInput demo[] = {
    {0,0,0,0}, {0,0,0,20}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {1,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {1,1,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {1,1,1,0}, {1,1,1,0}, {1,1,1,0},
    {1,1,1,0}, {1,1,1,0}, {1,1,1,0}, {1,1,0,0},
    {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,20}, {1,0,0,20}, {0,0,0,20}, {0,0,0,20}, {0,0,0,20},
    {0,0,0,20}, {0,0,0,20}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}
};
static DemoInput DemoSensorInput(void) {
    DemoInput empty = {false, false, false, 0};
    return g_tick < sizeof(demo) / sizeof(demo[0]) ? demo[g_tick] : empty;
}
bool Front_sensor_Interface(void) { return DemoSensorInput().front; }
bool Left_sensor_Interface(void) { return DemoSensorInput().left; }
bool Right_sensor_Interface(void) { return DemoSensorInput().right; }
int Dust_sensor_Interface(void) { return DemoSensorInput().dust; }

void Motor_Interface(DriveAction a) {
    static const char* n[] = {"IDLE","FORWARD","TURN_LEFT","TURN_RIGHT","BACKWARD"};
    g_interface_motor = a;
    printf("[MOTOR  ] %s\n", n[a]);
}
void Cleaner_Interface(CleanAction a) {
    static const char* n[] = {"OFF","NORMAL","POWER_UP"};
    g_interface_clean = a;
    printf("[CLEANER] %s\n", n[a]);
}

CleanAction Cleaner_Interface_GetLast(void) { return g_interface_clean; }
DriveAction Motor_Interface_GetLast(void) { return g_interface_motor; }

int main(void) {
    unsigned int i;
    static const char *drive_names[] = {"IDLE", "FORWARD", "LEFT", "RIGHT", "BACKWARD"};
    static const char *clean_names[] = {"OFF", "NORMAL", "POWER_UP"};
    printf("=== RVC Start (logical ticks; no real-time wait) ===\n");
    Controller_Init();
    for (i = 0; i < sizeof(demo) / sizeof(demo[0]); ++i) {
        RvcCommand cmd = Controller();
        DemoInput input = demo[i];
        printf("[STATE  ] tick=%u sensors=%d/%d/%d dust=%d motor=%s clean=%s\n",
            Controller_GetTick() - 1u, input.front, input.left, input.right,
            input.dust, drive_names[cmd.drive], clean_names[cmd.clean]);
        if (cmd.clean != Cleaner_Interface_GetLast() ||
            cmd.drive != Motor_Interface_GetLast()) {
            fprintf(stderr, "Interface state mismatch\n");
            return 1;
        }
    }
    printf("=== RVC End ===\n");
    return 0;
}

#endif /* RVC_TEST */
