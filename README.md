# RasberryPi 온도 기반 IR 자동 제어기

Raspberry Pi를 사용하여 DHT22 온습도 센서로 실내 온도를 측정하고, 설정된 온도에 따라 IR(적외선) 신호를 보내 선풍기나 에어컨 등의 가전제품을 자동으로 제어하는 프로젝트입니다. 모든 로그는 `log.csv` 파일에 기록됩니다.

---

## 📝 주요 기능

- **온도 기반 자동 제어**: 순수 히스테리시스 로직을 사용하여 특정 온도 이상에서 기기를 켜고, 특정 온도 이하에서 끕니다.
    - **ON**: 26.0°C 이상
    - **OFF**: 24.0°C 이하
- **안정적인 센서 읽기**: DHT22 센서의 값을 읽을 때 타임아웃 및 3회 재시도 로직을 적용하여 안정성을 높였습니다.
- **IR 원격 제어**: `pigpio` 라이브러리의 하드웨어 PWM을 사용하여 정확한 NEC 프로토콜 IR 신호를 생성합니다.
- **실시간 수동 제어**: 프로그램 실행 중 표준 입력(stdin)을 통해 `test`, `nec`, `car`와 같은 명령어로 즉시 기기를 제어할 수 있습니다.
- **CSV 로깅**: 모든 온도 측정, 기기 상태 변경, 오류 발생을 타임스탬프와 함께 `log.csv` 파일에 기록합니다.
- **실시간 스케줄링**: IR 신호와 같이 정밀한 타이밍이 요구되는 작업을 위해 `SCHED_FIFO` 실시간 스케줄러 사용을 시도합니다. (실패해도 프로그램은 계속 동작)

---

## ⚙️ 요구 사양

### 하드웨어

- Raspberry Pi 3B+ 또는 그 이상 모델
- DHT22 온습도 센서
- IR 발광 다이오드 (IR LED)
- 저항 및 브레드보드, 점퍼 와이어

### 소프트웨어

- Raspberry Pi OS (또는 다른 Linux 기반 OS)
- **pigpio** 라이브러리
- gcc / make

---

## 🔌 하드웨어 연결

| 컴포넌트 | Raspberry Pi 핀 (BCM) | 설명 |
| --- | --- | --- |
| **DHT22 DATA** | **GPIO 4** | 데이터 신호선 |
| **DHT22 VCC** | **5V** 또는 **3.3V** | 전원 |
| **DHT22 GND** | **GND** | 접지 |
| **IR LED (+)** | **GPIO 18** | `pigpio`의 하드웨어 PWM을 사용합니다. |
| **IR LED (-)** | **GND** | 접지 (LED 보호를 위해 적절한 저항을 직렬 연결) |

Sheets로 내보내기

---

## 🚀 설치 및 실행

### 1. pigpio 설치

`pigpio` 라이브러리가 설치되어 있지 않다면 먼저 설치합니다.

Bash

`sudo apt-get update
sudo apt-get install pigpio`

### 2. 소스 코드 수정 (필수)

`main.c` 파일을 열어 제어하려는 리모컨의 NEC 코드에 맞게 주소와 명령 코드를 수정해야 합니다.

C

`// ---- FILL THESE WITH YOUR CAPTURED CODES ----
#define NEC_ADDR 0x00       // <<< 제어할 리모컨의 주소(Address)로 변경#define NEC_CMD_TOGGLE 0x45 // <<< 제어할 리모컨의 전원 토글(Power Toggle) 명령(Command)으로 변경// ---------------------------------------------`

> 💡 팁: 리모컨의 IR 코드를 알아내려면 LIRC(Linux Infrared Remote Control) 패키지나 별도의 IR 수신 모듈을 사용하여 캡처할 수 있습니다.
> 

### 3. 빌드

다음 명령어를 사용하여 코드를 컴파일합니다. (`Makefile`이 없다면 직접 컴파일)

Bash

`# -lpigpio -lrt -lpthread 플래그를 사용하여 pigpio 라이브러리와 링크합니다.
gcc -Wall -o tempctrl main.c -lpigpio -lrt -lpthread`

> 또는, Makefile이 있다면 make 명령어만 실행하세요.
> 

### 4. 실행

`pigpio` 데몬을 먼저 실행한 후, `sudo` 권한으로 프로그램을 실행해야 합니다.

Bash

`# 1. pigpio 데몬 실행
sudo pigpiod

# 2. 프로그램 실행
sudo ./tempctrl`

프로그램이 시작되면 현재 설정값과 함께 온도 측정을 시작합니다. `Ctrl+C`를 눌러 종료할 수 있습니다.

---

## ⌨️ 실시간 명령어

프로그램이 실행되는 동안 터미널에 다음 명령어를 입력하고 Enter 키를 누르면 즉시 해당 동작을 수행합니다.

- `test` 또는 `nec`
    - 설정된 NEC 코드로 전원 토글(POWER TOGGLE) 신호를 한 번 보냅니다. 수동으로 기기를 켜거나 끌 때 사용합니다.
- `car`
    - 38kHz의 캐리어 신호를 300ms 동안 방출합니다. IR LED가 정상적으로 동작하는지 테스트할 때 유용합니다.

---

## 📊 로그 파일 (`log.csv`)

프로그램 실행 폴더에 `log.csv` 파일이 생성되며, 모든 동작이 기록됩니다.

**로그 형식:**`# timestamp,temperature_c,humidity_pct,fan_on,status`

**로그 예시:**

코드 스니펫

`# timestamp,temperature_c,humidity_pct,fan_on,status
2025-10-05 20:32:36,23.5,45.1,0,OK
2025-10-05 20:32:38,26.1,45.3,1,OK
2025-10-05 20:32:40,,,1,MANUAL_TOGGLE
2025-10-05 20:32:42,,,1,READ_FAIL(-4)`

- **status**:
    - `OK`: 정상 측정 및 제어
    - `MANUAL_TOGGLE`: 사용자가 명령어로 직접 상태 변경
    - `READ_FAIL(코드)`: 센서 읽기 실패 및 오류 코드
