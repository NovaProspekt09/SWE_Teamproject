#include "header.h"
#include <gtest/gtest.h>
#include <array>
#include <vector>

// Test interfaces retain outputs just like the simulated hardware.
struct SensorInput {
    bool front = false;
    bool left = false;
    bool right = false;
    int dust = 0;
};
enum class OutputKind { Motor, Cleaner };
struct CommandEvent { OutputKind kind; int value; };
static SensorInput sensors;
static DriveAction motor = ACT_IDLE;
static CleanAction cleaner = CLN_OFF;
static std::array<int, 4> sensor_reads;
static std::array<unsigned int, 4> sensor_ticks;
static std::array<int, 5> motor_calls;
static std::array<int, 3> cleaner_calls;
static std::vector<CommandEvent> events;

static void RecordRead(int index) {
    ++sensor_reads[index];
    sensor_ticks[index] = Controller_GetTick();
}
extern "C" {
bool Front_sensor_Interface(void) { RecordRead(0); return sensors.front; }
bool Left_sensor_Interface(void) { RecordRead(1); return sensors.left; }
bool Right_sensor_Interface(void) { RecordRead(2); return sensors.right; }
int Dust_sensor_Interface(void) { RecordRead(3); return sensors.dust; }
void Motor_Interface(DriveAction action) {
    motor = action;
    ++motor_calls[action];
    events.push_back({OutputKind::Motor, action});
}
void Cleaner_Interface(CleanAction action) {
    cleaner = action;
    ++cleaner_calls[action];
    events.push_back({OutputKind::Cleaner, action});
}
DriveAction Motor_Interface_GetLast(void) { return motor; }
CleanAction Cleaner_Interface_GetLast(void) { return cleaner; }
}

class RvcTest : public testing::Test {
protected:
    void SetUp() override {
        sensors = {};
        // Do not reset motor/cleaner in the test: initialization must do it.
        Controller_Init();
        ASSERT_EQ(motor, ACT_IDLE);
        ASSERT_EQ(cleaner, CLN_OFF);
        ClearCommands();
        sensor_reads.fill(0);
    }
    void ClearCommands() {
        events.clear();
        motor_calls.fill(0);
        cleaner_calls.fill(0);
    }
    RvcCommand Step() {
        unsigned int tick = Controller_GetTick();
        sensor_reads.fill(0);
        RvcCommand command = Controller();
        for (int i = 0; i < 4; ++i) {
            EXPECT_EQ(sensor_reads[i], 1) << "sensor " << i;
            EXPECT_EQ(sensor_ticks[i], tick) << "sensor " << i;
        }
        EXPECT_EQ(Controller_GetTick(), tick + 1u);
        EXPECT_EQ(command.drive, motor);
        EXPECT_EQ(command.clean, cleaner);
        EXPECT_EQ(Controller_GetLast().drive, command.drive);
        EXPECT_EQ(Controller_GetLast().clean, command.clean);
        return command;
    }
    void ExpectEvent(size_t index, OutputKind kind, int value) {
        ASSERT_LT(index, events.size());
        EXPECT_EQ(events[index].kind, kind);
        EXPECT_EQ(events[index].value, value);
    }
    void StartBackward() {
        sensors.front = sensors.left = sensors.right = true;
        EXPECT_EQ(Step().drive, ACT_MOVE_BACKWARD);
    }
};

TEST_F(RvcTest, SensorAllEightObstacleLocations) {
    const ObstacleLocation expected[] = {
        OBS_NONE, OBS_FRONT, OBS_LEFT, OBS_FRONT_LEFT,
        OBS_RIGHT, OBS_FRONT_RIGHT, OBS_LEFT_RIGHT, OBS_ALL
    };
    for (int mask = 0; mask < 8; ++mask) {
        SCOPED_TRACE(mask);
        sensors.front = (mask & 1) != 0;
        sensors.left = (mask & 2) != 0;
        sensors.right = (mask & 4) != 0;
        sensor_reads.fill(0);
        EXPECT_EQ(Determine_Obstacle_location(), expected[mask]);
        EXPECT_EQ(sensor_reads[0], 1);
        EXPECT_EQ(sensor_reads[1], 1);
        EXPECT_EQ(sensor_reads[2], 1);
        EXPECT_EQ(sensor_reads[3], 0);
        EXPECT_EQ(Controller_GetTick(), 0u);
    }
}
TEST_F(RvcTest, SensorDustThresholdIsTwenty) {
    const int values[] = {0, 19, 20, 70, 100};
    const DustLevel expected[] = {DUST_NONE, DUST_NONE, DUST_NORMAL, DUST_NORMAL, DUST_NORMAL};
    for (int i = 0; i < 5; ++i) {
        SCOPED_TRACE(values[i]);
        sensors.dust = values[i];
        sensor_reads.fill(0);
        EXPECT_EQ(Determine_dust_existence(), expected[i]);
        EXPECT_EQ(sensor_reads[3], 1);
        EXPECT_EQ(sensor_reads[0] + sensor_reads[1] + sensor_reads[2], 0);
    }
}
TEST_F(RvcTest, SensorReadsDoNotAdvanceTick) {
    for (int i = 0; i < 3; ++i) {
        Determine_Obstacle_location();
        Determine_dust_existence();
    }
    EXPECT_EQ(Controller_GetTick(), 0u);
    EXPECT_TRUE(events.empty());
}
TEST_F(RvcTest, ControllerSamplesEverySensorOnceAtTheSameTick) {
    for (int i = 0; i < 12; ++i) Step();
    EXPECT_EQ(Controller_GetTick(), 12u);
}
TEST_F(RvcTest, NoDustRunsForwardWithNormalCleaning) {
    RvcCommand result = Step();
    EXPECT_EQ(result.drive, ACT_MOVE_FORWARD);
    EXPECT_EQ(result.clean, CLN_NORMAL);
    ASSERT_EQ(events.size(), 2u);
    ExpectEvent(0, OutputKind::Motor, ACT_MOVE_FORWARD);
    ExpectEvent(1, OutputKind::Cleaner, CLN_NORMAL);
}
TEST_F(RvcTest, SideObstaclesDoNotStopForwardCleaning) {
    sensors.left = true;
    EXPECT_EQ(Step().drive, ACT_MOVE_FORWARD);
    sensors.left = false; sensors.right = true;
    EXPECT_EQ(Step().drive, ACT_MOVE_FORWARD);
    sensors.left = true;
    EXPECT_EQ(Step().drive, ACT_MOVE_FORWARD);
    EXPECT_EQ(cleaner, CLN_NORMAL);
    EXPECT_EQ(cleaner_calls[CLN_OFF], 0);
}
TEST_F(RvcTest, ForwardEnableAndNormalTriggerAreNotRepeated) {
    for (int i = 0; i < 10; ++i) Step();
    EXPECT_EQ(motor_calls[ACT_MOVE_FORWARD], 1);
    EXPECT_EQ(cleaner_calls[CLN_NORMAL], 1);
    EXPECT_EQ(events.size(), 2u);
}
TEST_F(RvcTest, FrontObstaclePrefersLeftWhenBothSidesAreOpen) {
    sensors.front = true;
    EXPECT_EQ(Step().drive, ACT_TURN_LEFT);
    EXPECT_EQ(cleaner, CLN_OFF);
}
TEST_F(RvcTest, FrontAndLeftObstacleSelectsRight) {
    sensors.front = sensors.left = true;
    EXPECT_EQ(Step().drive, ACT_TURN_RIGHT);
}
TEST_F(RvcTest, FrontAndRightObstacleSelectsLeft) {
    sensors.front = sensors.right = true;
    EXPECT_EQ(Step().drive, ACT_TURN_LEFT);
}
TEST_F(RvcTest, LeftTurnLastsFiveCyclesWithOneTrigger) {
    sensors.front = true;
    EXPECT_EQ(Step().drive, ACT_TURN_LEFT);
    sensors.front = false;
    for (int i = 1; i < 5; ++i) {
        EXPECT_EQ(Step().drive, ACT_TURN_LEFT);
        EXPECT_EQ(motor_calls[ACT_TURN_LEFT], 1);
        EXPECT_EQ(cleaner, CLN_OFF);
    }
    EXPECT_EQ(Step().drive, ACT_MOVE_FORWARD);
    EXPECT_EQ(cleaner, CLN_NORMAL);
}
TEST_F(RvcTest, RightTurnLastsFiveCyclesWithOneTrigger) {
    sensors.front = sensors.left = true;
    EXPECT_EQ(Step().drive, ACT_TURN_RIGHT);
    sensors.front = false;
    for (int i = 1; i < 5; ++i) {
        EXPECT_EQ(Step().drive, ACT_TURN_RIGHT);
        EXPECT_EQ(motor_calls[ACT_TURN_RIGHT], 1);
    }
    EXPECT_EQ(Step().drive, ACT_MOVE_FORWARD);
}
TEST_F(RvcTest, AvoidanceSendsOffBeforeDisableAndTurn) {
    Step(); ClearCommands(); sensors.front = true;
    Step();
    ASSERT_EQ(events.size(), 3u);
    ExpectEvent(0, OutputKind::Cleaner, CLN_OFF);
    ExpectEvent(1, OutputKind::Motor, ACT_IDLE);
    ExpectEvent(2, OutputKind::Motor, ACT_TURN_LEFT);
}
TEST_F(RvcTest, TurnRecoverySendsForwardBeforeNormalCleaning) {
    sensors.front = true; Step(); sensors.front = false;
    for (int i = 1; i < 5; ++i) Step();
    ClearCommands(); Step();
    ASSERT_EQ(events.size(), 2u);
    ExpectEvent(0, OutputKind::Motor, ACT_MOVE_FORWARD);
    ExpectEvent(1, OutputKind::Cleaner, CLN_NORMAL);
}
TEST_F(RvcTest, TurnRecoverySendsForwardBeforePowerUpCleaning) {
    sensors.front = sensors.left = true; Step(); sensors.front = false;
    sensors.dust = 20;
    for (int i = 1; i < 5; ++i) Step();
    ClearCommands(); Step();
    ASSERT_EQ(events.size(), 2u);
    ExpectEvent(0, OutputKind::Motor, ACT_MOVE_FORWARD);
    ExpectEvent(1, OutputKind::Cleaner, CLN_POWER_UP);
}
TEST_F(RvcTest, ConsecutiveAvoidanceDoesNotRestartCleaning) {
    Step(); sensors.front = true; Step();
    for (int i = 1; i < 5; ++i) Step();
    EXPECT_EQ(Step().drive, ACT_TURN_LEFT);
    EXPECT_EQ(motor_calls[ACT_TURN_LEFT], 2);
    EXPECT_EQ(cleaner_calls[CLN_OFF], 1);
    EXPECT_EQ(cleaner_calls[CLN_NORMAL], 1);
    sensors.left = sensors.right = true;
    for (int i = 1; i < 5; ++i) Step();
    EXPECT_EQ(Step().drive, ACT_MOVE_BACKWARD);
    EXPECT_EQ(cleaner, CLN_OFF);
    EXPECT_EQ(cleaner_calls[CLN_NORMAL], 1);
    EXPECT_EQ(cleaner_calls[CLN_OFF], 1);
}
TEST_F(RvcTest, AllDirectionsBlockedStartsBackwardWithCleaningOff) {
    sensors.dust = 20; Step(); ClearCommands(); StartBackward();
    EXPECT_EQ(cleaner, CLN_OFF);
    ASSERT_EQ(events.size(), 3u);
    ExpectEvent(0, OutputKind::Cleaner, CLN_OFF);
    ExpectEvent(1, OutputKind::Motor, ACT_IDLE);
    ExpectEvent(2, OutputKind::Motor, ACT_MOVE_BACKWARD);
}
TEST_F(RvcTest, BackwardCompletesMinimumThreeCyclesEvenIfASideOpensEarly) {
    StartBackward(); sensors.left = false;
    EXPECT_EQ(Step().drive, ACT_MOVE_BACKWARD);
    EXPECT_EQ(Step().drive, ACT_MOVE_BACKWARD);
    EXPECT_EQ(Step().drive, ACT_TURN_LEFT);
    EXPECT_EQ(motor_calls[ACT_MOVE_BACKWARD], 1);
}
TEST_F(RvcTest, BothSidesBlockedKeepBackwardEvenIfFrontClears) {
    StartBackward(); sensors.front = false; sensors.dust = 20;
    for (int i = 0; i < 15; ++i) {
        EXPECT_EQ(Step().drive, ACT_MOVE_BACKWARD);
        EXPECT_EQ(cleaner, CLN_OFF);
    }
    EXPECT_EQ(motor_calls[ACT_MOVE_BACKWARD], 1);
    EXPECT_EQ(motor_calls[ACT_TURN_LEFT] + motor_calls[ACT_TURN_RIGHT], 0);
    EXPECT_EQ(motor_calls[ACT_IDLE], 0);
    EXPECT_EQ(cleaner_calls[CLN_OFF], 0);
    EXPECT_EQ(cleaner_calls[CLN_POWER_UP], 0);
}
TEST_F(RvcTest, BackwardDisablesBeforeTakingAnOpenLeftSide) {
    StartBackward(); Step(); Step(); sensors.left = false;
    ClearCommands(); EXPECT_EQ(Step().drive, ACT_TURN_LEFT);
    ASSERT_EQ(events.size(), 2u);
    ExpectEvent(0, OutputKind::Motor, ACT_IDLE);
    ExpectEvent(1, OutputKind::Motor, ACT_TURN_LEFT);
}
TEST_F(RvcTest, BackwardDisablesBeforeTakingAnOpenRightSide) {
    StartBackward(); Step(); Step(); sensors.right = false;
    ClearCommands(); EXPECT_EQ(Step().drive, ACT_TURN_RIGHT);
    ASSERT_EQ(events.size(), 2u);
    ExpectEvent(0, OutputKind::Motor, ACT_IDLE);
    ExpectEvent(1, OutputKind::Motor, ACT_TURN_RIGHT);
}
TEST_F(RvcTest, BackwardPrefersLeftIfBothSidesBecomeOpen) {
    StartBackward(); Step(); Step(); sensors.left = sensors.right = false;
    EXPECT_EQ(Step().drive, ACT_TURN_LEFT);
}
TEST_F(RvcTest, BoostHoldsFiveCyclesAfterDustDisappears) {
    sensors.dust = 20; EXPECT_EQ(Step().clean, CLN_POWER_UP);
    sensors.dust = 0;
    for (int i = 1; i < 5; ++i) {
        EXPECT_EQ(Step().clean, CLN_POWER_UP);
        EXPECT_EQ(cleaner_calls[CLN_POWER_UP], 1);
        EXPECT_EQ(cleaner_calls[CLN_NORMAL], 0);
    }
    EXPECT_EQ(Step().clean, CLN_NORMAL);
    EXPECT_EQ(cleaner_calls[CLN_NORMAL], 1);
}
TEST_F(RvcTest, PersistentDustStartsANewBoostOnTheSixthCycle) {
    sensors.dust = 20;
    for (int i = 0; i < 11; ++i) {
        EXPECT_EQ(Step().clean, CLN_POWER_UP);
        EXPECT_EQ(cleaner_calls[CLN_POWER_UP], i / 5 + 1);
        EXPECT_EQ(cleaner_calls[CLN_NORMAL], 0);
    }
}
TEST_F(RvcTest, LateDustDetectionDoesNotExtendTheCurrentBoost) {
    sensors.dust = 20; Step(); sensors.dust = 0;
    Step(); Step(); Step();
    sensors.dust = 20; Step(); // The fifth cycle refreshes nothing.
    sensors.dust = 0;
    EXPECT_EQ(Step().clean, CLN_NORMAL);
    EXPECT_EQ(cleaner_calls[CLN_POWER_UP], 1);
}
TEST_F(RvcTest, AvoidanceInterruptsBoostAndRetainsOffWithoutRepeatedCalls) {
    sensors.dust = 20; Step(); sensors.front = true; Step();
    sensors.front = false;
    for (int i = 1; i < 5; ++i) {
        EXPECT_EQ(Step().clean, CLN_OFF);
        EXPECT_EQ(cleaner_calls[CLN_OFF], 1);
        EXPECT_EQ(cleaner_calls[CLN_POWER_UP], 1);
    }
    sensors.dust = 0;
    EXPECT_EQ(Step().clean, CLN_NORMAL);
}
TEST_F(RvcTest, AvoidanceRecoveryStartsAFreshFullBoost) {
    sensors.dust = 20; Step(); sensors.front = true; Step(); sensors.front = false;
    for (int i = 1; i < 5; ++i) Step();
    EXPECT_EQ(Step().clean, CLN_POWER_UP);
    sensors.dust = 0;
    for (int i = 1; i < 5; ++i) EXPECT_EQ(Step().clean, CLN_POWER_UP);
    EXPECT_EQ(Step().clean, CLN_NORMAL);
    EXPECT_EQ(cleaner_calls[CLN_POWER_UP], 2);
}
TEST_F(RvcTest, RetainedCleanerModeDoesNotRequireAnotherCommand) {
    sensors.dust = 20; Step(); ClearCommands(); sensors.dust = 0;
    for (int i = 1; i < 5; ++i) {
        Step();
        EXPECT_TRUE(events.empty());
        EXPECT_EQ(Cleaner_Interface_GetLast(), CLN_POWER_UP);
    }
}
TEST_F(RvcTest, ReinitializationSendsOffThenIdleWithoutResettingTestOutputs) {
    sensors.dust = 20; Step(); ClearCommands();
    ASSERT_EQ(cleaner, CLN_POWER_UP); ASSERT_EQ(motor, ACT_MOVE_FORWARD);
    Controller_Init();
    ASSERT_EQ(events.size(), 2u);
    ExpectEvent(0, OutputKind::Cleaner, CLN_OFF);
    ExpectEvent(1, OutputKind::Motor, ACT_IDLE);
    EXPECT_EQ(cleaner, CLN_OFF); EXPECT_EQ(motor, ACT_IDLE);
    EXPECT_EQ(Controller_GetTick(), 0u);
    EXPECT_EQ(Controller_GetLast().drive, ACT_IDLE);
    EXPECT_EQ(Controller_GetLast().clean, CLN_OFF);
    ClearCommands(); sensors.dust = 0;
    EXPECT_EQ(Step().clean, CLN_NORMAL);
    EXPECT_EQ(motor_calls[ACT_MOVE_FORWARD], 1);
}
TEST_F(RvcTest, ReinitializationClearsTurnBackwardAndBoostTimers) {
    for (int state = 0; state < 4; ++state) {
        SCOPED_TRACE(state);
        sensors = {}; Controller_Init(); sensors.dust = 20; Step(); Step();
        if (state > 0) {
            sensors.front = true; sensors.left = state >= 2; sensors.right = state == 3;
            Step();
        }
        // Deliberately keep actual latched outputs when restarting.
        Controller_Init(); ClearCommands(); sensors = {};
        sensors.dust = 20;
        for (int i = 0; i < 5; ++i) {
            EXPECT_EQ(Step().drive, ACT_MOVE_FORWARD);
            EXPECT_EQ(cleaner, CLN_POWER_UP);
            EXPECT_EQ(cleaner_calls[CLN_POWER_UP], 1);
        }
        sensors.dust = 0;
        EXPECT_EQ(Step().clean, CLN_NORMAL);
        EXPECT_EQ(motor_calls[ACT_MOVE_FORWARD], 1);
    }
}
TEST_F(RvcTest, ReinitializeThenImmediateBackwardCannotLeaveBoostRunning) {
    sensors.dust = 20; Step(); Controller_Init(); StartBackward();
    EXPECT_EQ(cleaner, CLN_OFF);
    EXPECT_EQ(Controller_GetLast().clean, Cleaner_Interface_GetLast());
}
TEST_F(RvcTest, ReinitializeDuringTurnAllowsAFreshFiveCycleTurn) {
    for (int direction = 0; direction < 2; ++direction) {
        SCOPED_TRACE(direction);
        sensors = {};
        Controller_Init();
        sensors.front = true;
        sensors.left = direction == 1;
        Step(); Step();

        Controller_Init(); ClearCommands();
        sensors.left = direction == 0;
        DriveAction new_turn = direction == 0 ? ACT_TURN_RIGHT : ACT_TURN_LEFT;
        EXPECT_EQ(Step().drive, new_turn);
        ASSERT_EQ(events.size(), 1u);
        ExpectEvent(0, OutputKind::Motor, new_turn);
        sensors.front = false;
        for (int i = 1; i < 5; ++i) {
            EXPECT_EQ(Step().drive, new_turn);
            EXPECT_EQ(cleaner, CLN_OFF);
            EXPECT_EQ(motor_calls[new_turn], 1);
        }
        EXPECT_EQ(Step().drive, ACT_MOVE_FORWARD);
        EXPECT_EQ(cleaner, CLN_NORMAL);
    }
}
TEST_F(RvcTest, ReinitializeDuringBackwardStartsAFreshMinimumThreeCycles) {
    StartBackward(); Step();
    Controller_Init(); ClearCommands();
    StartBackward();
    sensors.left = false;
    EXPECT_EQ(Step().drive, ACT_MOVE_BACKWARD);
    EXPECT_EQ(Step().drive, ACT_MOVE_BACKWARD);
    EXPECT_EQ(Step().drive, ACT_TURN_LEFT);
    EXPECT_EQ(motor_calls[ACT_MOVE_BACKWARD], 1);
    EXPECT_EQ(motor_calls[ACT_IDLE], 1);
    EXPECT_EQ(cleaner, CLN_OFF);
}
