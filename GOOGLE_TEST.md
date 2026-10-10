# RVC GoogleTest

`test_controller.cpp`는 실제 GoogleTest의 `TEST_F`와 `EXPECT_EQ` 등을 사용하는
33개 테스트입니다.

테스트에서는 `controller.c`를 C17로 컴파일하고 `RVC_TEST`를 정의합니다.
센서와 모터·청소기 인터페이스만 테스트용으로 교체하며, 판단·이동·청소 모듈과
Controller는 실제 구현을 실행합니다. 테스트 코드는 C++17입니다.

## Visual Studio에서 실행

1. 처음 실행할 때는 아래 PowerShell 명령으로 GoogleTest를 다운로드합니다.
2. 이 폴더의 `RvcGoogleTests.vcxproj`를 Visual Studio에서 엽니다.
3. 구성을 **Debug / x64**로 선택합니다.
4. `RvcGoogleTests` 프로젝트를 시작 프로젝트로 지정합니다.
5. **Ctrl+F5**를 누릅니다. 콘솔 마지막의 통과·실패 개수를 확인합니다.

테스트 프로젝트는 저장소의 `controller.c`와 `header.h`를 직접 사용합니다.
GoogleTest의 진입점은 일반 실행용 `main()`과 분리되어 있습니다.

GoogleTest v1.18.0은 아래 스크립트가 `third_party/googletest`에 다운로드합니다.
Git과 Visual Studio의 C++ 데스크톱 개발 도구를 설치한 뒤,
이 폴더에서 다음 명령을 실행하면 라이브러리 다운로드·빌드·테스트를 수행합니다.
시스템 실행 정책은 변경하지 않으며 실행하는 PowerShell 프로세스에만 적용합니다.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_google_tests.ps1
```

스크립트는 GoogleTest 버전과 커밋을 확인하고 결과를 다음 파일에 저장합니다.

- `test-results/google-test-results.xml`: 테스트별 통과·실패 결과
- `test-results/google-test-console.txt`: 콘솔 출력

이미 빌드한 실행 파일을 직접 실행할 수도 있습니다.

```powershell
.\build\google-tests\x64\Debug\bin\rvc_google_tests.exe
.\build\google-tests\x64\Debug\bin\rvc_google_tests.exe --gtest_filter="RvcTest.*Boost*"
```

## 시나리오

| 범위 | 개수 | 확인 내용 |
|---|---:|---|
| 센서·틱 | 4 | 장애물 8조합, 먼지 기준 19/20, 센서 호출 자체는 틱을 변경하지 않음, 같은 틱에서 센서별 한 번 읽기 |
| 평상시 이동 | 3 | 먼지가 없으면 전진·일반 청소, 옆 장애물만 있으면 전진 유지, 같은 명령 반복 없음 |
| 방향 선택·회전 | 9 | 왼쪽 우선, 막힌 방향에 따른 선택, 좌·우회전 각각 5틱, Off 선행, 회피 종료 후 전진 선행, 연속 회피 |
| 후진 | 6 | 모두 막혔을 때 후진, 최소 3틱 유지, 양옆이 막혔으면 후진 지속, 열린 방향으로 전환, 후진 Disable 후 회전 |
| 강화 청소 | 6 | 총 5틱 유지, 6번째 틱 재선택, 도중 먼지 재감지로 연장하지 않음, 회피 우선, 회피 후 새 5틱, 마지막 명령 유지 |
| 재초기화 | 5 | 실제 Off·Idle 명령 전달, 상태·틱 초기화, 바로 후진해도 청소 Off, 새 회전·후진의 정상 시작 |

각 Controller 실행을 검증하는 `Step()`은 추가로 다음을 검사합니다.

- 앞·왼쪽·오른쪽·먼지 센서를 각각 정확히 한 번 읽었는가?
- 모든 센서를 동일한 틱 번호에서 읽었는가?
- 실행 후 틱이 정확히 한 번 증가했는가?
- 반환한 명령, Controller에 저장된 명령, 인터페이스가 유지하는 상태가 일치하는가?

## 중요한 시간·우선순위 검사

| 상황 | 기대 결과 | 테스트 이름 |
|---|---|---|
| 강화 시작 직후 먼지 사라짐 | 시작 틱을 포함해 5틱은 강화, 6번째 틱에 일반 청소 | `BoostHoldsFiveCyclesAfterDustDisappears` |
| 먼지가 계속 있음 | 1·6·11번째 틱에만 강화 명령 | `PersistentDustStartsANewBoostOnTheSixthCycle` |
| 5번째 틱에 먼지 재감지, 6번째 틱에 없음 | 유지 시간 연장 없이 일반 청소로 전환 | `LateDustDetectionDoesNotExtendTheCurrentBoost` |
| 강화 중 장애물 발견 | Off로 전환, 회피 중 청소 명령 반복 없음 | `AvoidanceInterruptsBoostAndRetainsOffWithoutRepeatedCalls` |
| 회피 종료 시 먼지 있음 | 기존 타이머와 무관한 새 5틱 강화 | `AvoidanceRecoveryStartsAFreshFullBoost` |
| 회피 시작 | Off → 전진 Disable(Idle) → 회전 | `AvoidanceSendsOffBeforeDisableAndTurn` |
| 회피 종료 | 전진 → 현재 먼지에 따른 일반/강화 청소 | `TurnRecoverySendsForwardBeforeNormalCleaning`, `TurnRecoverySendsForwardBeforePowerUpCleaning` |
| 후진 시작 후 옆이 일찍 열림 | 최초 3틱은 후진, 4번째 틱에 회전 | `BackwardCompletesMinimumThreeCyclesEvenIfASideOpensEarly` |
| 후진 중 앞만 열림 | 양옆이 막혀 있으면 후진 유지 | `BothSidesBlockedKeepBackwardEvenIfFrontClears` |
| 회전 중 초기화 후 다른 회전 요청 | 새 회전 명령을 한 번 보내고 새로 5틱 유지 | `ReinitializeDuringTurnAllowsAFreshFiveCycleTurn` |

33은 서로 이름이 다른 테스트 개수입니다. 반복문에서 수행하는 조건 검사 수를
별도의 테스트 개수로 세지 않습니다. 이 테스트는 가상 인터페이스를 사용하므로
실제 센서·모터의 물리적 동작이나 실제 경과 시간을 검증하는 것은 아닙니다.

## 실행 확인 (2026-10-10)

- Visual Studio MSVC v145, Debug / x64에서 GoogleTest 프로젝트 빌드 성공
- GoogleTest 33개 통과, 실패·비활성·오류 각각 0개
- 실행 순서를 섞어 두 번 반복했을 때도 매번 33개 통과
- 위 결과의 XML과 콘솔 출력은 `test-results` 폴더에 저장
