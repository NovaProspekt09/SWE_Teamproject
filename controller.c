/* ============================================================
 *  RVC (Robot Vacuum Cleaner) - Structured Design Implementation
 *  Team Project #2 / Software Engineering
 *  Build (run)  : gcc -std=c17 -o rvc controller.c
 *  Build (test) : g++ -DRVC_TEST controller.c test_rvc.cpp -lgtest_main -lgtest
 * ============================================================ */
#include <stdio.h>
#include "header.h"

/* ------------------ Internal State ------------------ */
static RvcCommand g_last;
static DriveAction g_drive;
static int g_motion_ticks;
static int g_power_up_ticks;
#ifndef RVC_TEST
static unsigned int g_tick;
#endif

void Controller_Init(void) {
    g_last.drive = ACT_IDLE;
    g_last.clean = CLN_OFF;
    g_drive = ACT_MOVE_FORWARD;
    g_motion_ticks = 0;
    g_power_up_ticks = 0;
}

/* ============== Afferent Flow (Input) =============== */
ObstacleLocation Determine_Obstacle_Location(bool f, bool l, bool r) {
    if (f && l && r) return OBS_ALL;
    if (f && l)      return OBS_FRONT_LEFT;
    if (f && r)      return OBS_FRONT_RIGHT;
    if (l && r)      return OBS_LEFT_RIGHT;
    if (f)           return OBS_FRONT;
    if (l)           return OBS_LEFT;
    if (r)           return OBS_RIGHT;
    return OBS_NONE;
}

DustLevel Determine_Dust_Existence(int dust_raw) {
    /* Keep the existing raw-value categories; any detected dust boosts cleaning. */
    if (dust_raw >= 70) return DUST_HEAVY;
    if (dust_raw >= 20) return DUST_NORMAL;
    return DUST_NONE;
}

bool Dust_Existence(DustLevel lv) { return lv != DUST_NONE; }

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

RvcCommand Controller_Step(ObstacleLocation obs, DustLevel dust) {
    RvcCommand cmd;

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

    /* Avoidance overrides dust. Normal forward travel always cleans.
     * New dust detections refresh the boost timer; avoidance cancels it. */
    if (cmd.drive != ACT_MOVE_FORWARD) {
        g_power_up_ticks = 0;
        cmd.clean = CLN_OFF;
    } else {
        if (Dust_Existence(dust)) g_power_up_ticks = RVC_POWER_UP_TICKS;
        if (g_power_up_ticks > 0) {
            cmd.clean = CLN_POWER_UP;
            --g_power_up_ticks;
        } else {
            cmd.clean = CLN_NORMAL;
        }
    }

    switch (cmd.drive) {
    case ACT_MOVE_FORWARD: Move_Forward_Module(); break;
    case ACT_MOVE_BACKWARD: Move_Backward_Module(); break;
    case ACT_TURN_LEFT: Turn_Left_Module(); break;
    case ACT_TURN_RIGHT: Turn_Right_Module(); break;
    default: break;
    }
    switch (cmd.clean) {
    case CLN_POWER_UP: Power_Up_Clean_Module(); break;
    case CLN_NORMAL: Cleaner_Normal_Module(); break;
    case CLN_OFF: Dust_Clean_Off_Module(); break;
    }
    g_last = cmd;
    return cmd;
}

RvcCommand Controller_GetLast(void) { return g_last; }

/* Retain the public startup function name for existing callers.
 * Startup means initialization, not unconditional boosted cleaning. */
void Controller_PowerUpSequence(void) {
    Controller_Init();
    Move_Forward_Module();
    Cleaner_Normal_Module();
    g_last.drive = ACT_MOVE_FORWARD;
    g_last.clean = CLN_NORMAL;
}

/* ===================== Main Loop ===================== */
void Rvc_Tick(void) {
    ObstacleLocation obs = Determine_Obstacle_Location(
        Front_Sensor_Interface(),
        Left_Sensor_Interface(),
        Right_Sensor_Interface());
    DustLevel dust = Determine_Dust_Existence(Dust_Sensor_Interface());
    Controller_Step(obs, dust);
}

static void wait(int ms) { (void)ms; /* 200ms 주기 에뮬레이션 */ }

int Rvc_Main(int max_tick) {
    #ifndef RVC_TEST
    g_tick = 0;
#endif
    Controller_PowerUpSequence();
    for (int i = 0; max_tick < 0 || i < max_tick; ) {
        Rvc_Tick();
        wait(200);
        if (max_tick >= 0) ++i;
    }
    return 0;
}

/* ============================================================
 *  아래 블록은 실행 빌드 전용.
 *  GoogleTest 빌드 시 -DRVC_TEST 로 제외 → 테스트 더블이 대체함.
 * ============================================================ */
#ifndef RVC_TEST

/* ---- Library Modules: HW Emulation ---- */


bool Front_Sensor_Interface(void) { return (g_tick % 7) == 3; }
bool Left_Sensor_Interface(void)  { return (g_tick % 11) == 5; }
bool Right_Sensor_Interface(void) { return (g_tick % 13) == 8; }
int  Dust_Sensor_Interface(void)  { g_tick++; return (int)(((g_tick % 100u) * 17u) % 100u); }

void Motor_Interface(DriveAction a) {
    static const char* n[] = {"IDLE","FORWARD","TURN_LEFT","TURN_RIGHT","BACKWARD"};
    printf("[MOTOR  ] %s\n", n[a]);
}
void Cleaner_Interface(CleanAction a) {
    static const char* n[] = {"OFF","NORMAL","POWER_UP"};
    printf("[CLEANER] %s\n", n[a]);
}

int main(void) {
    printf("=== RVC Start ===\n");
    Rvc_Main(20);          /* 실제 제품에서는 Rvc_Main(-1) */
    printf("=== RVC End ===\n");
    return 0;
}

#endif /* RVC_TEST */