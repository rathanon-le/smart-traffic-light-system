#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

// ================= OLED =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ================= Pins =================
static const int CAR_R = 25;
static const int CAR_Y = 27;
static const int CAR_G = 26;

static const int BTN_PIN = 4;      // INPUT_PULLUP (pressed = LOW)

// ✅ PIR 2 ตัว
static const int PIR1_PIN = 32;
static const int PIR2_PIN = 34;

static const int BUZZER_PIN = 13;  // buzzer

// ================= Event bits =================
static const EventBits_t EVT_CROSS_REQ    = (1 << 0);
static const EventBits_t EVT_CROSS_ACTIVE = (1 << 1);
static const EventBits_t EVT_ILLEGAL      = (1 << 2);

// ================= Timing (ms) =================
static const uint32_t GREEN_CROSS_MS  = 10000;

// PIR filter/poll
static const uint32_t PIR_CONFIRM_MS  = 200;
static const uint32_t PIR_POLL_MS     = 80;

// ✅ ต้องตรวจจับทั้ง 2 ตัวภายในช่วงเวลานี้ถึงจะ illegal
static const uint32_t BOTH_WINDOW_MS  = 800;

// ✅ หลังเจอ illegal: รันเตือน 3s แล้วหยุดทันที จากนั้นพักตรวจใหม่ 5s
static const uint32_t ILLEGAL_RUN_MS  = 3000;
static const uint32_t ILLEGAL_WAIT_MS = 5000;

// buzzer pattern during illegal
static const uint32_t BEEP_ON_MS  = 120;
static const uint32_t BEEP_OFF_MS = 180;

// ================= RTOS Objects =================
static EventGroupHandle_t gEvent = nullptr;
static SemaphoreHandle_t  gI2CMutex = nullptr;

// ✅ ช่วงเวลาที่ “พัก/ignore” การตรวจ illegal (หลังจบ illegal 5s)
static volatile uint32_t gIllegalIgnoreUntil = 0;

// ================= Helpers =================
static void setCar(bool r, bool y, bool g) {
  digitalWrite(CAR_R, r);
  digitalWrite(CAR_Y, y);
  digitalWrite(CAR_G, g);
}

static void oledShow(const char* l1, const char* l2 = "", const char* l3 = "") {
  if (!gI2CMutex) return;
  if (xSemaphoreTake(gI2CMutex, pdMS_TO_TICKS(200)) != pdTRUE) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(l1);
  if (l2[0]) display.println(l2);
  if (l3[0]) display.println(l3);
  display.display();

  xSemaphoreGive(gI2CMutex);
}

static void beepOnce(uint16_t onMs, uint16_t offMs) {
  digitalWrite(BUZZER_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(onMs));
  digitalWrite(BUZZER_PIN, LOW);
  vTaskDelay(pdMS_TO_TICKS(offMs));
}

static bool confirmHigh(int pin, uint32_t confirmMs) {
  if (digitalRead(pin) == LOW) return false;
  uint32_t t0 = millis();
  while (digitalRead(pin) == HIGH && (millis() - t0) < confirmMs) {
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  return (millis() - t0) >= confirmMs;
}

// ================= Tasks =================

// ---- ButtonTask: debounce + กัน request ซ้อน ----
static void TaskButton(void*) {
  Serial.println("[ButtonTask] started");

  bool lastStable = HIGH;
  bool lastRead   = HIGH;
  uint32_t lastChangeMs = 0;

  while (1) {
    bool r = digitalRead(BTN_PIN);

    if (r != lastRead) {
      lastRead = r;
      lastChangeMs = millis();
    }

    if ((millis() - lastChangeMs) > 50 && r != lastStable) {
      lastStable = r;

      if (lastStable == LOW) {
        EventBits_t bits = xEventGroupGetBits(gEvent);

        if (bits & EVT_CROSS_ACTIVE) {
          Serial.println("[ButtonTask] ignored (locked during crossing)");
        } else if (bits & EVT_ILLEGAL) {
          Serial.println("[ButtonTask] ignored (illegal active)");
        } else {
          Serial.println("[ButtonTask] press -> EVT_CROSS_REQ");
          xEventGroupSetBits(gEvent, EVT_CROSS_REQ);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---- PIRTask: ✅ ต้อง PIR 2 ตัวภายใน window ถึง set EVT_ILLEGAL
//               ✅ และหลัง illegal จบ จะ ignore 5 วินาที ----
static void TaskPIR(void*) {
  Serial.println("[PIRTask] started (warm-up 30-60s)");

  uint32_t tPir1 = 0;
  uint32_t tPir2 = 0;

  while (1) {
    uint32_t now = millis();
    EventBits_t bits = xEventGroupGetBits(gEvent);

    // ✅ ระหว่าง legal crossing: บล็อค illegal
    if (bits & EVT_CROSS_ACTIVE) {
      xEventGroupClearBits(gEvent, EVT_ILLEGAL);
      tPir1 = tPir2 = 0;
      vTaskDelay(pdMS_TO_TICKS(PIR_POLL_MS));
      continue;
    }

    // ✅ หลัง illegal จบ: พักตรวจใหม่ 5 วินาที
    if (now < gIllegalIgnoreUntil) {
      xEventGroupClearBits(gEvent, EVT_ILLEGAL);
      tPir1 = tPir2 = 0;
      vTaskDelay(pdMS_TO_TICKS(PIR_POLL_MS));
      continue;
    }

    bool m1 = confirmHigh(PIR1_PIN, PIR_CONFIRM_MS);
    bool m2 = confirmHigh(PIR2_PIN, PIR_CONFIRM_MS);

    if (m1) tPir1 = now;
    if (m2) tPir2 = now;

    bool both = (tPir1 > 0 && tPir2 > 0 &&
                 (uint32_t)abs((int32_t)tPir1 - (int32_t)tPir2) <= BOTH_WINDOW_MS);

    if (both) {
      xEventGroupSetBits(gEvent, EVT_ILLEGAL);
    } else {
      // เคลียร์เมื่อ timestamp เก่าเกิน window ทั้งคู่
      bool stale1 = (tPir1 == 0) || (now - tPir1 > BOTH_WINDOW_MS);
      bool stale2 = (tPir2 == 0) || (now - tPir2 > BOTH_WINDOW_MS);

      if (stale1 && stale2) {
        xEventGroupClearBits(gEvent, EVT_ILLEGAL);
        tPir1 = tPir2 = 0;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(PIR_POLL_MS));
  }
}

// ---- TrafficTask: legal crossing = ล็อคไฟอื่นจนเวลาครบ
//                   illegal = รันเตือน 3s แล้วหยุด + พักตรวจ 5s ----
enum TrafficState { ST_RED_IDLE, ST_GREEN_CROSS, ST_ILLEGAL };

static void TaskTraffic(void*) {
  Serial.println("[TrafficTask] started");

  TrafficState st = ST_RED_IDLE;
  uint32_t crossStart = 0;
  uint32_t lastOled = 0;

  setCar(true, false, false);
  oledShow("MODE: RED (IDLE)", "Press button", "Car: RED");

  while (1) {
    uint32_t now = millis();
    EventBits_t bits = xEventGroupGetBits(gEvent);
    bool illegal = (bits & EVT_ILLEGAL);

    // ✅ illegal ทำงานได้เฉพาะตอน "ไม่ล็อค"
    if (illegal && !(bits & EVT_CROSS_ACTIVE) && st != ST_GREEN_CROSS) {
      st = ST_ILLEGAL;

      // กัน request ค้าง
      xEventGroupClearBits(gEvent, EVT_CROSS_REQ);

      Serial.println("[Traffic] ILLEGAL -> run 3s then stop");

      // ----- รันคำสั่งเตือน 3 วินาที -----
      uint32_t t0 = millis();
      while (millis() - t0 < ILLEGAL_RUN_MS) {
        setCar(false, true, false); // เหลืองค้าง

        if (millis() - lastOled > 200) {
          oledShow("WARNING!", "Illegal crossing", "3s alert...");
          lastOled = millis();
        }

        beepOnce(BEEP_ON_MS, BEEP_OFF_MS); // beep ต่อเนื่องในช่วง 3s
      }

      // ----- หยุดทันที -----
      digitalWrite(BUZZER_PIN, LOW);

      // เคลียร์ illegal bit + กลับแดง
      xEventGroupClearBits(gEvent, EVT_ILLEGAL);
      st = ST_RED_IDLE;
      setCar(true, false, false);
      oledShow("ALERT STOP", "Back to RED", "Wait 5s");

      // ✅ พักตรวจใหม่ 5 วินาที
      gIllegalIgnoreUntil = millis() + ILLEGAL_WAIT_MS;

      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // ---------- NORMAL states ----------
    switch (st) {
      case ST_RED_IDLE: {
        setCar(true, false, false);

        if (bits & EVT_CROSS_REQ) {
          xEventGroupClearBits(gEvent, EVT_CROSS_REQ);
          xEventGroupSetBits(gEvent, EVT_CROSS_ACTIVE);

          st = ST_GREEN_CROSS;
          crossStart = millis();

          oledShow("LEGAL CROSSING", "Car: GREEN", "LOCKED");
          break;
        }

        if (millis() - lastOled > 400) {
          if (now < gIllegalIgnoreUntil) {
            uint32_t left = (gIllegalIgnoreUntil - now + 999) / 1000;
            String l2 = "Ignore illegal: " + String(left) + "s";
            oledShow("MODE: RED (IDLE)", l2.c_str(), "Car: RED");
          } else {
            oledShow("MODE: RED (IDLE)", "Press button", "Car: RED");
          }
          lastOled = millis();
        }
        break;
      }

      case ST_GREEN_CROSS: {
        // ✅ ระหว่าง crossing: บังคับให้มีไฟเขียวอย่างเดียว (บล็อคไฟอื่น)
        setCar(false, false, true);

        uint32_t elapsed = millis() - crossStart;
        uint32_t leftMs = (elapsed >= GREEN_CROSS_MS) ? 0 : (GREEN_CROSS_MS - elapsed);
        uint32_t secLeft = (leftMs + 999) / 1000;

        if (millis() - lastOled > 250) {
          String line2 = "Green: " + String(secLeft) + "s";
          oledShow("LEGAL CROSSING", line2.c_str(), "LOCKED");
          lastOled = millis();
        }

        if (elapsed >= GREEN_CROSS_MS) {
          xEventGroupClearBits(gEvent, EVT_CROSS_ACTIVE);
          st = ST_RED_IDLE;
          setCar(true, false, false);
          oledShow("DONE", "Back to RED", "UNLOCK");
          vTaskDelay(pdMS_TO_TICKS(200));
        }
        break;
      }

      case ST_ILLEGAL:
      default:
        // handled above
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ================= Setup =================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(CAR_R, OUTPUT);
  pinMode(CAR_Y, OUTPUT);
  pinMode(CAR_G, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BTN_PIN, INPUT_PULLUP);

  // PIR inputs (ถ้ามี false trigger ให้ใช้ INPUT_PULLDOWN หรือใส่ R10k pull-down)
  pinMode(PIR1_PIN, INPUT);
  pinMode(PIR2_PIN, INPUT);

  gEvent = xEventGroupCreate();
  gI2CMutex = xSemaphoreCreateMutex();

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[OLED] init failed");
  } else {
    oledShow("SYSTEM READY", "2-PIR illegal", "Press button");
  }

  xTaskCreatePinnedToCore(TaskButton,  "ButtonTask",  2048, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(TaskPIR,     "PIRTask",     4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(TaskTraffic, "TrafficTask", 4096, nullptr, 2, nullptr, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}