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
static const int PIR_PIN = 32;     // PIR 1 ตัว
static const int BUZZER_PIN = 13;  // buzzer

// ================= Event bits =================
static const EventBits_t EVT_CROSS_REQ    = (1 << 0); // button pressed
static const EventBits_t EVT_CROSS_ACTIVE = (1 << 1); // green window active (legal crossing)
static const EventBits_t EVT_ILLEGAL      = (1 << 2); // illegal detected (latched)

// ================= Timing (ms) =================
static const uint32_t GREEN_CROSS_MS = 10000; // ✅ เวลาที่ให้ไฟเขียวตอนขอข้าม
static const uint32_t PIR_CONFIRM_MS  = 200;  // confirm กัน false
static const uint32_t PIR_COOLDOWN_MS = 200;  // อ่านถี่ขึ้นเพื่อรู้ว่า illegal จบเมื่อไหร่

// ================= RTOS Objects =================
static EventGroupHandle_t gEvent = nullptr;
static SemaphoreHandle_t  gI2CMutex = nullptr;

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

    // debounce 50ms
    if ((millis() - lastChangeMs) > 50 && r != lastStable) {
      lastStable = r;

      if (lastStable == LOW) {
        EventBits_t bits = xEventGroupGetBits(gEvent);

        // ถ้า illegal อยู่ ให้ไม่รับ request
        if (bits & EVT_ILLEGAL) {
          Serial.println("[ButtonTask] ignored (illegal active)");
        }
        // ถ้าอยู่ช่วงข้ามอยู่แล้ว ไม่รับซ้ำ
        else if (bits & EVT_CROSS_ACTIVE) {
          Serial.println("[ButtonTask] ignored (cross active)");
        } else {
          Serial.println("[ButtonTask] press -> EVT_CROSS_REQ");
          xEventGroupSetBits(gEvent, EVT_CROSS_REQ);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---- PIRTask: ถ้า detect -> set ILLEGAL, ถ้าไม่ detect -> clear ILLEGAL ----
static void TaskPIR(void*) {
  Serial.println("[PIRTask] started (warm-up 30-60s)");

  while (1) {
    bool motion = confirmHigh(PIR_PIN, PIR_CONFIRM_MS);

    if (motion) {
      xEventGroupSetBits(gEvent, EVT_ILLEGAL);
    } else {
      // ถ้าอ่านแล้วไม่ detect ให้เคลียร์ illegal
      xEventGroupClearBits(gEvent, EVT_ILLEGAL);
    }

    vTaskDelay(pdMS_TO_TICKS(PIR_COOLDOWN_MS));
  }
}

// ---- TrafficTask: RED ค้างจน request, request -> GREEN window, illegal -> YELLOW + beep ----
enum TrafficState { ST_RED_IDLE, ST_GREEN_CROSS, ST_ILLEGAL };
static TrafficState st = ST_RED_IDLE;

static void TaskTraffic(void*) {
  Serial.println("[TrafficTask] started");

  uint32_t crossStart = 0;
  uint32_t lastOled = 0;

  // เริ่มต้น: RED ค้าง
  setCar(true, false, false);
  oledShow("MODE: RED (IDLE)", "Press button", "Car Can go");

  while (1) {
    EventBits_t bits = xEventGroupGetBits(gEvent);
    bool illegal = (bits & EVT_ILLEGAL);

    // ---------- illegal override ----------
    if (illegal) {
      // เข้าโหมด ILLEGAL และยกเลิกช่วงข้าม (ถ้ามี)
      if (st != ST_ILLEGAL) {
        st = ST_ILLEGAL;
        xEventGroupClearBits(gEvent, EVT_CROSS_ACTIVE);
        xEventGroupClearBits(gEvent, EVT_CROSS_REQ);
        Serial.println("[Traffic] -> ILLEGAL mode");
      }

      setCar(false, true, false); // เหลืองค้าง

      if (millis() - lastOled > 250) {
        oledShow("WARNING!", "Illegal crossing", "Car: YELLOW");
        lastOled = millis();
      }

      // ส่งเสียงร้องเป็นจังหวะ “ต่อเนื่อง” ระหว่าง illegal
      beepOnce(120, 180);

      // ไม่ไปทำ state อื่น
      continue;
    }

    // ---------- illegal ended ----------
    if (st == ST_ILLEGAL && !illegal) {
      // เมื่อ illegal จบ -> กลับไป RED idle เสมอ
      st = ST_RED_IDLE;
      setCar(true, false, false);
      oledShow("ILLEGAL END", "Back to RED", "Car can go ");
      Serial.println("[Traffic] illegal end -> RED idle");
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // ---------- normal states ----------
    switch (st) {
      case ST_RED_IDLE: {
        setCar(true, false, false);

        // ถ้ามี request -> เริ่ม GREEN window
        if (bits & EVT_CROSS_REQ) {
          xEventGroupClearBits(gEvent, EVT_CROSS_REQ);
          xEventGroupSetBits(gEvent, EVT_CROSS_ACTIVE);

          st = ST_GREEN_CROSS;
          crossStart = millis();

          oledShow("LEGAL CROSSING", "Car: GREEN", "Time: 10s");
          Serial.println("[Traffic] request -> GREEN window");
          break;
        }

        if (millis() - lastOled > 400) {
          oledShow("MODE: RED (IDLE)", "Press button", "Car can go");
          lastOled = millis();
        }
        break;
      }

      case ST_GREEN_CROSS: {
        setCar(false, false, true);

        uint32_t elapsed = millis() - crossStart;
        uint32_t leftMs = (elapsed >= GREEN_CROSS_MS) ? 0 : (GREEN_CROSS_MS - elapsed);
        uint32_t secLeft = (leftMs + 999) / 1000;

        if (millis() - lastOled > 250) {
          oledShow("LEGAL CROSSING", ("Green: " + String(secLeft) + "s").c_str(), "Car: GREEN");
          lastOled = millis();
        }

        // หมดเวลา -> กลับ RED idle
        if (elapsed >= GREEN_CROSS_MS) {
          xEventGroupClearBits(gEvent, EVT_CROSS_ACTIVE);
          st = ST_RED_IDLE;
          setCar(true, false, false);
          oledShow("DONE", "Back to RED", "Car can go");
          Serial.println("[Traffic] GREEN done -> RED idle");
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
  Serial.println("\n=== Smart Traffic Light (RED idle, request->GREEN, illegal->YELLOW+beep) ===");

  // Outputs
  pinMode(CAR_R, OUTPUT);
  pinMode(CAR_Y, OUTPUT);
  pinMode(CAR_G, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Inputs
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(PIR_PIN, INPUT);

  // RTOS objects
  gEvent = xEventGroupCreate();
  gI2CMutex = xSemaphoreCreateMutex();

  // OLED
  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[OLED] init failed (check wiring/address)");
  } else {
    oledShow("SYSTEM READY", "RED idle mode", "Press button");
    Serial.println("[OLED] init OK (0x3C)");
  }

  // Create tasks
  xTaskCreatePinnedToCore(TaskButton,  "ButtonTask",  2048, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(TaskPIR,     "PIRTask",     3072, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(TaskTraffic, "TrafficTask",  4096, nullptr, 2, nullptr, 1);

  Serial.println("=== Tasks created. System running. ===");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
