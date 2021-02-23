#pragma once

#if defined(WiFi_h) && !defined(ENABLE_WIFI)
//#define ENABLE_WIFI
#endif

#include <M5Atom.h>

#include <algorithm>
#include <vector>

#ifdef ENABLE_WIFI
#include <StreamString.h>
#endif

#include "quickjs.h"
#include "bmpfont5x5.h"

#define MATRIX_WIDTH  5 // LEDマトリクスの幅
#define BUTTON_A   0
#define DIMENSION_X   0
#define DIMENSION_Y   1
#define DIMENSION_Z   2
#define DIMENSION_STRENGTH   3
#define IMU_MULTIPLE  100
#define LED_COLOR_ON  0x00ff00
#define LED_COLOR_OFF 0x000000
#define MAX_TEXT_LENGTH 10

typedef struct PIN_INFO{
  const char* name;
  uint8_t pin;
} PIN_INFO;

const PIN_INFO analog_pin_list[] = {
  { "P0", GPIO_NUM_0 },
  { "P1", GPIO_NUM_1 }
};
const PIN_INFO digital_pin_list[] = {
  { "P0", GPIO_NUM_0 },
  { "P1", GPIO_NUM_1 }
};

typedef struct ICON_INFO{
  const char* name;
  uint8_t bmp[(MATRIX_WIDTH + 7) / 8 * MATRIX_WIDTH];
} ICON_INFO;

const ICON_INFO icon_list[] = {
  { "Heart", { 0x0a, 0x1f, 0x1f, 0x0e, 0x04 } },
  { "SmallHeart", { 0x00, 0x0a, 0x0e, 0x04, 0x00 } },
  { "Yes", { 0x00, 0x01, 0x02, 0x14, 0x08 } },
  { "No", { 0x11, 0x0a, 0x04, 0x0a, 0x11 } },
  { "Happy", { 0x00, 0x0a, 0x00, 0x11, 0x0e } },
  { "Sad", { 0x00, 0x0a, 0x00, 0x0e, 0x11 } },
  { "Confused", { 0x00, 0x0a, 0x00, 0x0a, 0x15 } },
  { "Angry", { 0x11, 0x0a, 0x00, 0x1f, 0x15 } },
  { "Asleep", { 0x00, 0x1b, 0x00, 0x0e, 0x00 } },
  { "Surprised", { 0x0a, 0x00, 0x04, 0x0a, 0x04 } },
  { "Silly", { 0x11, 0x00, 0x1f, 0x03, 0x03 } },
  { "Fabulous", { 0x1f, 0x0b, 0x00, 0x0a, 0x0e } },
  { "Meh", { 0x0b, 0x00, 0x02, 0x04, 0x08 } },
  { "TShirt", { 0x0b, 0x1f, 0x0e, 0x0e, 0x0e } },
  { "Rollerskate", { 0x03, 0x03, 0x1f, 0x1f, 0x0a } },
  { "Duck", { 0x0c, 0x1c, 0x0f, 0x0e, 0x00 } },
  { "House", { 0x04, 0x0e, 0x1f, 0x0e, 0x0a } },
  { "Tortoise", { 0x00, 0x0e, 0x1f, 0x0a, 0x00 } },
  { "Butterfly", { 0x1b, 0x1f, 0x04, 0x1f, 0x1b } },
  { "StickFigure", { 0x04, 0x1f, 0x04, 0x0a, 0x11 } },
  { "Ghost", { 0x0e, 0x15, 0x1f, 0x1f, 0x15 } },
  { "Sword", { 0x04, 0x04, 0x04, 0x0e, 0x04 } },
  { "Giraffe", { 0x18, 0x08, 0x08, 0x0e, 0x0a } },
  { "Skull", { 0x0e, 0x15, 0x1f, 0x0e, 0x0e } },
  { "Umbrella", { 0x0e, 0x1f, 0x04, 0x14, 0x1c } },
  { "Snake", { 0x18, 0x1b, 0x0a, 0x0e, 0x00 } },
  { "Rabbit", { 0x14, 0x14, 0x1e, 0x1a, 0x1e } },
  { "Cow", { 0x11, 0x11, 0x1f, 0x0e, 0x04 } },
  { "QuarterNote", { 0x04, 0x04, 0x04, 0x1a, 0x1a } },
  { "EigthNote", 0x04, 0x06, 0x05, 0x1c, 0x1c },
  { "Pitchfork", { 0x15, 0x15, 0x1f, 0x04, 0x04 } },
  { "Target", { 0x04, 0x0e, 0x1b, 0x0e, 0x04 } },
  { "Triangle", { 0x00, 0x04, 0x0a, 0x1f, 0x00 } },
  { "LeftTriangle", { 0x10, 0x18, 0x14, 0x12, 0x1f } },
  { "Chessboard", 0x0a, 0x15, 0x0a, 0x15, 0x0a },
  { "Diamond", 0x04, 0x0a, 0x11, 0x0a, 0x04 },
  { "SmallDiamond", 0x00, 0x04, 0x0a, 0x04, 0x00 },
  { "Square", { 0x1f, 0x11, 0x11, 0x11, 0x1f } },
  { "SmallSquare", { 0x00, 0x0e, 0x0a, 0x0e, 0x00 } },
  { "Scissors", { 0x19, 0x1a, 0x04, 0x1a, 0x19 } },
};

static void qjs_dump_exception(JSContext *ctx, JSValue v) {
  if (!JS_IsUndefined(v)) {
    const char *str = JS_ToCString(ctx, v);
    if (str) {
      Serial.println(str);
      JS_FreeCString(ctx, str);
    } else {
      Serial.println("[Exception]");
    }
  }
  JSValue e = JS_GetException(ctx);
  const char *str = JS_ToCString(ctx, e);
  if (str) {
    Serial.println(str);
    JS_FreeCString(ctx, str);
  }
  if (JS_IsError(ctx, e)) {
    JSValue s = JS_GetPropertyStr(ctx, e, "stack");
    if (!JS_IsUndefined(s)) {
      const char *str = JS_ToCString(ctx, s);
      if (str) {
        Serial.println(str);
        JS_FreeCString(ctx, str);
      }
    }
    JS_FreeValue(ctx, s);
  }
  JS_FreeValue(ctx, e);
}

#ifdef ENABLE_WIFI
class JSHttpFetcher {
  struct Entry {
    HTTPClient *client;
    JSValue resolving_funcs[2];
    int status;
    void result(JSContext *ctx, uint32_t func, JSValue body) {
      delete client;  // dispose connection before invoke;
      JSValue r = JS_NewObject(ctx);
      JS_SetPropertyStr(ctx, r, "body", JS_DupValue(ctx, body));
      JS_SetPropertyStr(ctx, r, "status", JS_NewInt32(ctx, status));
      JS_Call(ctx, resolving_funcs[func], JS_UNDEFINED, 1, &r);
      JS_FreeValue(ctx, r);
      JS_FreeValue(ctx, resolving_funcs[0]);
      JS_FreeValue(ctx, resolving_funcs[1]);
    }
  };
  std::vector<Entry *> queue;

 public:
  JSValue fetch(JSContext *ctx, JSValueConst jsUrl, JSValueConst options) {
    if (WiFi.status() != WL_CONNECTED) {
      return JS_EXCEPTION;
    }
    const char *url = JS_ToCString(ctx, jsUrl);
    if (!url) {
      return JS_EXCEPTION;
    }
    const char *method = nullptr, *body = nullptr;
    if (JS_IsObject(options)) {
      JSValue m = JS_GetPropertyStr(ctx, options, "method");
      if (JS_IsString(m)) {
        method = JS_ToCString(ctx, m);
      }
      JSValue b = JS_GetPropertyStr(ctx, options, "body");
      if (JS_IsString(m)) {
        body = JS_ToCString(ctx, b);
      }
    }

    Entry *ent = new Entry();
    ent->client = new HTTPClient();
    ent->client->begin(url);
    JS_FreeCString(ctx, url);

    // TODO: remove blocking calls.
    if (method) {
      ent->status = ent->client->sendRequest(method, (uint8_t *)body,
                                             body ? strlen(body) : 0);
    } else {
      ent->status = ent->client->GET();
    }
    queue.push_back(ent);

    JS_FreeCString(ctx, method);
    JS_FreeCString(ctx, body);
    return JS_NewPromiseCapability(ctx, ent->resolving_funcs);
  }

  void loop(JSContext *ctx) {
    int doneCount = 0;
    for (auto &pent : queue) {
      WiFiClient *stream = pent->client->getStreamPtr();
      if (stream == nullptr || pent->status <= 0) {
        // reject.
        pent->result(ctx, 1, JS_UNDEFINED);
        delete pent;
        pent = nullptr;
        doneCount++;
        continue;
      }
      if (stream->available()) {
        String body = pent->client->getString();
        JSValue bodyStr = JS_NewString(ctx, body.c_str());
        body.clear();
        pent->result(ctx, 0, bodyStr);
        JS_FreeValue(ctx, bodyStr);
        delete pent;
        pent = nullptr;
        doneCount++;
      }
    }

    if (doneCount > 0) {
      queue.erase(std::remove_if(queue.begin(), queue.end(),
                                 [](Entry *pent) { return pent == nullptr; }),
                  queue.end());
    }
  }
};
#endif  // ENABLE_WIFI

class JSTimer {
  // 20 bytes / entry.
  struct TimerEntry {
    uint32_t id;
    int32_t timeout;
    int32_t interval;
    JSValue func;
  };
  std::vector<TimerEntry> timers;
  uint32_t id_counter = 0;

 public:
  uint32_t RegisterTimer(JSValue f, int32_t time, int32_t interval = -1) {
    uint32_t id = ++id_counter;
    timers.push_back(TimerEntry{id, time, interval, f});
    return id;
  }
  void RemoveTimer(uint32_t id) {
    timers.erase(std::remove_if(timers.begin(), timers.end(),
                                [id](TimerEntry &t) { return t.id == id; }),
                 timers.end());
  }
  int32_t GetNextTimeout(int32_t now) {
    if (timers.empty()) {
      return -1;
    }
    std::sort(timers.begin(), timers.end(),
              [now](TimerEntry &a, TimerEntry &b) -> bool {
                return (a.timeout - now) >
                       (b.timeout - now);  // 2^32 wraparound
              });
    int next = timers.back().timeout - now;
    return max(next, 0);
  }
  bool ConsumeTimer(JSContext *ctx, int32_t now) {
    std::vector<TimerEntry> t;
    int32_t eps = 2;
    while (!timers.empty() && timers.back().timeout - now <= eps) {
      t.push_back(timers.back());
      timers.pop_back();
    }
    for (auto &ent : t) {
      // NOTE: may update timers in this JS_Call().
      JSValue r = JS_Call(ctx, ent.func, ent.func, 0, nullptr);
      if (JS_IsException(r)) {
        qjs_dump_exception(ctx, r);
      }
      JS_FreeValue(ctx, r);

      if (ent.interval >= 0) {
        ent.timeout = now + ent.interval;
        timers.push_back(ent);
      } else {
        JS_FreeValue(ctx, ent.func);
      }
    }
    return !t.empty();
  }
};

class ESP32QuickJS {
 public:
  JSRuntime *rt;
  JSContext *ctx;
  JSTimer timer;
  JSValue loop_func = JS_UNDEFINED;
  JSValue btn_func = JS_UNDEFINED;
#ifdef ENABLE_WIFI
  JSHttpFetcher httpFetcher;
#endif

  void begin() {
    JSRuntime *rt = JS_NewRuntime();
    begin(rt, JS_NewContext(rt));
  }

  void begin(JSRuntime *rt, JSContext *ctx, int memoryLimit = 0) {
    this->rt = rt;
    this->ctx = ctx;
    if (memoryLimit == 0) {
      memoryLimit = ESP.getFreeHeap() >> 1;
    }
    JS_SetMemoryLimit(rt, memoryLimit);
    JS_SetGCThreshold(rt, memoryLimit >> 3);
    JSValue global = JS_GetGlobalObject(ctx);
    setup(ctx, global);
    JS_FreeValue(ctx, global);
  }

  void end() {
    JS_FreeContext(ctx);
    JS_FreeRuntime(rt);
  }

  void loop(bool callLoopFn = true) {
    // async
    JSContext *c;
    int ret = JS_ExecutePendingJob(JS_GetRuntime(ctx), &c);
    if (ret < 0) {
      qjs_dump_exception(ctx, JS_UNDEFINED);
    }

    // timer
    uint32_t now = millis();
    if (timer.GetNextTimeout(now) >= 0) {
      timer.ConsumeTimer(ctx, now);
    }

#ifdef ENABLE_WIFI
    httpFetcher.loop(ctx);
#endif

    // loop()
    if (callLoopFn && JS_IsFunction(ctx, loop_func)) {
      JSValue ret = JS_Call(ctx, loop_func, loop_func, 0, nullptr);
      if (JS_IsException(ret)) {
        qjs_dump_exception(ctx, ret);
      }
      JS_FreeValue(ctx, ret);
    }

    if (M5.Btn.wasReleased() && JS_IsFunction(ctx, btn_func)) {
      JSValue ret = JS_Call(ctx, btn_func, btn_func, 0, nullptr);
      if (JS_IsException(ret)) {
        qjs_dump_exception(ctx, ret);
      }
      JS_FreeValue(ctx, ret);
    }
  }

  void runGC() { JS_RunGC(rt); }

  bool exec(const char *code) {
    JSValue result = eval(code);
    bool ret = JS_IsException(result);
    JS_FreeValue(ctx, result);
    return ret;
  }

  JSValue eval(const char *code) {
    JSValue ret =
        JS_Eval(ctx, code, strlen(code), "<eval>", JS_EVAL_TYPE_MODULE);
    if (JS_IsException(ret)) {
      qjs_dump_exception(ctx, ret);
    }
    return ret;
  }

  void setLoopFunc(const char *fname) {
    JSValue global = JS_GetGlobalObject(ctx);
    setLoopFunc(JS_GetPropertyStr(ctx, global, fname));
    JS_FreeValue(ctx, global);
  }

  int load_module(const void *buf, int buf_len, const char *filename)
  {
    int ret = 0;

    /* for the modules, we compile then run to be able to set import.meta */
    JSValue val = JS_Eval(this->ctx, (const char*)buf, buf_len, filename,
                  JS_EVAL_TYPE_MODULE | JS_EVAL_FLAG_COMPILE_ONLY);
    if (!JS_IsException(val)) {
//              js_module_set_import_meta(ctx, val, TRUE, TRUE);
      val = JS_EvalFunction(this->ctx, val);
    }
    if (JS_IsException(val)) {
      qjs_dump_exception(this->ctx, val);
      ret = -1;
    }
    JS_FreeValue(this->ctx, val);

    // JSMemoryUsage usage;
    // JS_ComputeMemoryUsage(this->rt, &usage);
    // Serial.printf("malloc_size=%ld, malloc_limit=%ld, memory_usage_size=%ld\n", usage.malloc_size, usage.malloc_limit, usage.memory_used_size);

    // uint32_t size = ESP.getFreeHeap();
    // Serial.printf("FreeHeap=%d\n", size);

    return ret;
  }

 protected:

  void setLoopFunc(JSValue f) {
    JS_FreeValue(ctx, loop_func);
    loop_func = f;
  }

  void setBtnFunc(JSValue f) {
    JS_FreeValue(ctx, btn_func);
    btn_func = f;
  }

  virtual void setup(JSContext *ctx, JSValue global) {
    this->ctx = ctx;
    JS_SetContextOpaque(ctx, this);

    // setup console.log()
    JSValue console = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, global, "console", console);
    JS_SetPropertyStr(ctx, console, "log",
                      JS_NewCFunction(ctx, console_log, "log", 1));

    JSValue IconNames = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, global, "IconNames", IconNames);
    for( int i = 0 ; i < sizeof(icon_list) / sizeof(icon_list[0]); i++ ){
      JS_SetPropertyStr(ctx, IconNames, icon_list[i].name, JS_NewInt32(ctx, i));
    }

    JSValue basic = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, global, "basic", basic);
    JS_SetPropertyStr(ctx, basic, "forever",
                      JS_NewCFunction(ctx, basic_forever, "forever", 1));
    JS_SetPropertyStr(ctx, basic, "showLeds",
                      JS_NewCFunction(ctx, basic_showLeds, "showLeds", 1));
    JS_SetPropertyStr(ctx, basic, "pause",
                      JS_NewCFunction(ctx, basic_pause, "pause", 1));
    JS_SetPropertyStr(ctx, basic, "clearScreen",
                      JS_NewCFunction(ctx, basic_clearScreen, "clearScreen", 0));
    JS_SetPropertyStr(ctx, basic, "showNumber",
                      JS_NewCFunction(ctx, basic_showNumber, "showNumber", 1));
    JS_SetPropertyStr(ctx, basic, "showString",
                      JS_NewCFunction(ctx, basic_showString, "showString", 1));
    JS_SetPropertyStr(ctx, basic, "showIcon",
                      JS_NewCFunction(ctx, basic_showIcon, "showIcon", 1));

    JSValue Button = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, global, "Button", Button);
    JS_SetPropertyStr(ctx, Button, "A", JS_NewInt32(ctx, BUTTON_A));

    JSValue Dimension = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, global, "Dimension", Dimension);
    JS_SetPropertyStr(ctx, Dimension, "X", JS_NewInt32(ctx, DIMENSION_X));
    JS_SetPropertyStr(ctx, Dimension, "Y", JS_NewInt32(ctx, DIMENSION_Y));
    JS_SetPropertyStr(ctx, Dimension, "Z", JS_NewInt32(ctx, DIMENSION_Z));
    JS_SetPropertyStr(ctx, Dimension, "Strength", JS_NewInt32(ctx, DIMENSION_STRENGTH));

    JSValue input = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, global, "input", input);
    JS_SetPropertyStr(ctx, input, "onButtonPressed",
                      JS_NewCFunction(ctx, input_onButtonPressed, "onButtonPressed", 2));
    JS_SetPropertyStr(ctx, input, "buttonIsPressed",
                      JS_NewCFunction(ctx, input_buttonIsPressed, "buttonIsPressed", 1));
    JS_SetPropertyStr(ctx, input, "acceleration",
                      JS_NewCFunction(ctx, input_acceleration, "acceleration", 1));

    JSValue led = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, global, "led", led);
    JS_SetPropertyStr(ctx, led, "plot",
                      JS_NewCFunction(ctx, led_plot, "plot", 2));
    JS_SetPropertyStr(ctx, led, "unplot",
                      JS_NewCFunction(ctx, led_unplot, "unplot", 2));

    JSValue control = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, global, "control", control);
    JS_SetPropertyStr(ctx, control, "millis",
                      JS_NewCFunction(ctx, control_millis, "millis", 0));

    JSValue DigitalPin = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, global, "DigitalPin", DigitalPin);
    for( int i = 0 ; i < sizeof(digital_pin_list) / sizeof(digital_pin_list[0]) ; i++ ){
      JS_SetPropertyStr(ctx, DigitalPin, digital_pin_list[i].name, JS_NewInt32(ctx, digital_pin_list[i].pin));
    }
    JSValue AnalogPin = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, global, "AnalogPin", AnalogPin);
    for( int i = 0 ; i < sizeof(analog_pin_list) / sizeof(analog_pin_list[0]) ; i++ ){
      JS_SetPropertyStr(ctx, AnalogPin, analog_pin_list[i].name, JS_NewInt32(ctx, analog_pin_list[i].pin));
    }

    JSValue pins = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, global, "pins", pins);
    JS_SetPropertyStr(ctx, pins, "digitalReadPin",
                      JS_NewCFunction(ctx, pins_digitalReadPin, "digitalReadPin", 1));
    JS_SetPropertyStr(ctx, pins, "digitalWritePin",
                      JS_NewCFunction(ctx, pins_digitalWritePin, "digitalWritePin", 2));
    JS_SetPropertyStr(ctx, pins, "analogReadPin",
                      JS_NewCFunction(ctx, pins_analogReadPin, "analogReadPin", 1));

    JSValue Math = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, global, "Math", Math);
    JS_SetPropertyStr(ctx, Math, "digitalReadPin",
                      JS_NewCFunction(ctx, Math_max, "max", 2));
    JS_SetPropertyStr(ctx, Math, "digitalReadPin",
                      JS_NewCFunction(ctx, Math_min, "min", 2));
    JS_SetPropertyStr(ctx, Math, "digitalReadPin",
                      JS_NewCFunction(ctx, Math_abs, "abs", 1));
    JS_SetPropertyStr(ctx, Math, "randint",
                      JS_NewCFunction(ctx, Math_randint, "randint", 2));
    JS_SetPropertyStr(ctx, Math, "randomBoolean",
                      JS_NewCFunction(ctx, Math_randomBoolean, "randomBoolean", 0));
    JS_SetPropertyStr(ctx, Math, "constrain",
                      JS_NewCFunction(ctx, Math_constrain, "constrain", 3));
    JS_SetPropertyStr(ctx, Math, "sqrt",
                      JS_NewCFunction(ctx, Math_sqrt, "sqrt", 1));
    JS_SetPropertyStr(ctx, Math, "sin",
                      JS_NewCFunction(ctx, Math_sin, "sin", 1));
    JS_SetPropertyStr(ctx, Math, "cos",
                      JS_NewCFunction(ctx, Math_cos, "cos", 1));
    JS_SetPropertyStr(ctx, Math, "tan",
                      JS_NewCFunction(ctx, Math_tan, "tan", 1));

    // timer
    JS_SetPropertyStr(ctx, global, "setTimeout",
                      JS_NewCFunction(ctx, set_timeout, "setTimeout", 2));
    JS_SetPropertyStr(ctx, global, "clearTimeout",
                      JS_NewCFunction(ctx, clear_timeout, "clearTimeout", 1));
    JS_SetPropertyStr(ctx, global, "setInterval",
                      JS_NewCFunction(ctx, set_interval, "setInterval", 2));
    JS_SetPropertyStr(ctx, global, "clearInterval",
                      JS_NewCFunction(ctx, clear_timeout, "clearInterval", 1));

    static const JSCFunctionListEntry esp32_funcs[] = {
        JSCFunctionListEntry{"millis", 0, JS_DEF_CFUNC, 0, {
                               func : {0, JS_CFUNC_generic, esp32_millis}
                             }},
        JSCFunctionListEntry{"deepSleep", 0, JS_DEF_CFUNC, 0, {
                               func : {1, JS_CFUNC_generic, esp32_deep_sleep}
                             }},
        JSCFunctionListEntry{"restart", 0, JS_DEF_CFUNC, 0, {
                               func : {1, JS_CFUNC_generic, esp32_restart}
                             }},
        JSCFunctionListEntry{"delay", 0, JS_DEF_CFUNC, 0, {
                               func : {1, JS_CFUNC_generic, esp32_delay}
                             }},
        JSCFunctionListEntry{"setLoop", 0, JS_DEF_CFUNC, 0, {
                               func : {1, JS_CFUNC_generic, esp32_set_loop}
                             }},
#ifdef ENABLE_WIFI
        JSCFunctionListEntry{"isWifiConnected", 0, JS_DEF_CFUNC, 0, {
                               func : {0, JS_CFUNC_generic, wifi_is_connected}
                             }},
        JSCFunctionListEntry{"fetch", 0, JS_DEF_CFUNC, 0, {
                               func : {2, JS_CFUNC_generic, http_fetch}
                             }},
#endif
    };

#ifndef GLOBAL_ESP32
    JSModuleDef *mod;
    mod = JS_NewCModule(ctx, "esp32", [](JSContext *ctx, JSModuleDef *m) {
          return JS_SetModuleExportList(
              ctx, m, esp32_funcs,
              sizeof(esp32_funcs) / sizeof(JSCFunctionListEntry));
        });
    if (mod) {
      JS_AddModuleExportList(
          ctx, mod, esp32_funcs,
          sizeof(esp32_funcs) / sizeof(JSCFunctionListEntry));
    }

#else
    // import * as esp32 from "esp32";
    JSValue esp32 = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, global, "esp32", esp32);
    JS_SetPropertyFunctionList(
        ctx, esp32, esp32_funcs,
        sizeof(esp32_funcs) / sizeof(JSCFunctionListEntry));
#endif
  }

  static JSValue console_log(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    for (int i = 0; i < argc; i++) {
      const char *str = JS_ToCString(ctx, argv[i]);
      if (str) {
        Serial.println(str);
        JS_FreeCString(ctx, str);
      }
    }
    return JS_UNDEFINED;
  }

  static JSValue basic_forever(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    ESP32QuickJS *qjs = (ESP32QuickJS *)JS_GetContextOpaque(ctx);
    qjs->timer.RegisterTimer(JS_DupValue(ctx, argv[0]), millis(), 0);
    return JS_UNDEFINED;
  }

  static JSValue basic_showLeds(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    const char *str = JS_ToCString(ctx, argv[0]);
    if (!str)
      return JS_UNDEFINED;
    
    int index = 0;
    while( *str != '\0' ){
      if( *str != '.' && *str != '#'){
        str++;
        continue;
      }
      if( *str == '#'){
        M5.dis.drawpix(index, LED_COLOR_ON);
      }else if( *str == '.' ){
        M5.dis.drawpix(index, LED_COLOR_OFF);
      }
      str++;
      index++;
    }

    return JS_UNDEFINED;
  }

  static JSValue basic_pause(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    uint32_t value;
    JS_ToUint32(ctx, &value, argv[0]);

    delay(value);

    return JS_UNDEFINED;
  }

  static JSValue basic_clearScreen(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    uint8_t buffer[2 + 3 * MATRIX_WIDTH * MATRIX_WIDTH] = { MATRIX_WIDTH, MATRIX_WIDTH };
    for( int y = 0 ; y < MATRIX_WIDTH ; y++ ){
      for( int x = 0 ; x < MATRIX_WIDTH ; x++ ){
        buffer[2 + y * MATRIX_WIDTH * 3 + x * 3] = 0x000000;
        buffer[2 + y * MATRIX_WIDTH * 3 + x * 3 + 1] = 0x000000;
        buffer[2 + y * MATRIX_WIDTH * 3 + x * 3 + 2] = 0x000000;
      }
    }

    M5.dis.displaybuff(buffer);

    return JS_UNDEFINED;
  }

  static JSValue basic_showNumber(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    uint32_t value;
    JS_ToUint32(ctx, &value, argv[0]);

    char text[2];
    text[0] = '0' + value;
    text[0] = '\n';
    setText(text, 3);

    return JS_UNDEFINED;
  }

  static JSValue basic_showString(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    const char *str = JS_ToCString(ctx, argv[0]);
    if (str) {
      setText(str, 3);
      JS_FreeCString(ctx, str);
    }

    return JS_UNDEFINED;
  }

  static JSValue basic_showIcon(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    uint32_t value;
    JS_ToUint32(ctx, &value, argv[0]);

    uint8_t buffer[2 + 3 * MATRIX_WIDTH * MATRIX_WIDTH] = { MATRIX_WIDTH, MATRIX_WIDTH };
    const uint8_t *bmp = icon_list[value].bmp;
    for( int y = 0 ; y < MATRIX_WIDTH ; y++ ){
      for( int x = 0 ; x < MATRIX_WIDTH ; x++ ){
        if( (bmp[y] >> (MATRIX_WIDTH - x - 1)) & 0x01 ){
          buffer[2 + y * MATRIX_WIDTH * 3 + x * 3] = (LED_COLOR_ON >> 8) & 0xff;
          buffer[2 + y * MATRIX_WIDTH * 3 + x * 3 + 1] = (LED_COLOR_ON >> 16) & 0xff;
          buffer[2 + y * MATRIX_WIDTH * 3 + x * 3 + 2] = (LED_COLOR_ON >> 0) & 0xff;
        }
      }
    }

    M5.dis.displaybuff(buffer);

    return JS_UNDEFINED;
  }

  static JSValue input_onButtonPressed(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    uint32_t value;
    JS_ToUint32(ctx, &value, argv[0]);
    if( value == BUTTON_A ){
      ESP32QuickJS *qjs = (ESP32QuickJS *)JS_GetContextOpaque(ctx);
      qjs->setBtnFunc(JS_DupValue(ctx, argv[1]));
    }

    return JS_UNDEFINED;
  }

  static JSValue input_buttonIsPressed(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    uint32_t value;
    JS_ToUint32(ctx, &value, argv[0]);
    if( value == BUTTON_A ){
      return JS_NewBool(ctx, M5.Btn.isPressed());
    }

    return JS_UNDEFINED;
  }
  
  static JSValue input_acceleration(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    uint32_t value;
    JS_ToUint32(ctx, &value, argv[0]);
    float ax, ay, az;
    M5.IMU.getAccelData(&ax, &ay, &az);

    if( value == DIMENSION_X ){
      return JS_NewInt32(ctx, (int32_t)(ax * IMU_MULTIPLE));
    }else if( value == DIMENSION_Y ){
      return JS_NewInt32(ctx, (int32_t)(ay * IMU_MULTIPLE));
    }else if( value == DIMENSION_Z ){
      return JS_NewInt32(ctx, (int32_t)(az * IMU_MULTIPLE));
    }else if( value == DIMENSION_STRENGTH ){
      return JS_NewInt32(ctx, (int32_t)(sqrt(ax * ax + ay * ay + az * az) * IMU_MULTIPLE));
    }

    return JS_UNDEFINED;
  }

  static JSValue led_plot(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    uint32_t value1;
    JS_ToUint32(ctx, &value1, argv[0]);
    uint32_t value2;
    JS_ToUint32(ctx, &value2, argv[1]);

    M5.dis.drawpix(value2 * MATRIX_WIDTH + value1, LED_COLOR_ON);

    return JS_UNDEFINED;
  }

  static JSValue led_unplot(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    uint32_t value1;
    JS_ToUint32(ctx, &value1, argv[0]);
    uint32_t value2;
    JS_ToUint32(ctx, &value2, argv[1]);

    M5.dis.drawpix(value2 * MATRIX_WIDTH + value1, LED_COLOR_OFF);

    return JS_UNDEFINED;
  }

  static JSValue control_millis(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    return JS_NewUint32(ctx, millis());
  }

  static JSValue pins_digitalReadPin(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    uint32_t value;
    JS_ToUint32(ctx, &value, argv[0]);

    pinMode(value, INPUT);
    int p = digitalRead(value);

    return JS_NewInt32(ctx, p);
  }

  static JSValue Math_max(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    int32_t value1;
    JS_ToInt32(ctx, &value1, argv[0]);
    int32_t value2;
    JS_ToInt32(ctx, &value2, argv[1]);

    return JS_NewInt32(ctx, min(value1, value2));
  }

  static JSValue Math_min(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    int32_t value1;
    JS_ToInt32(ctx, &value1, argv[0]);
    int32_t value2;
    JS_ToInt32(ctx, &value2, argv[1]);

    return JS_NewInt32(ctx, max(value1, value2));
  }

  static JSValue Math_abs(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    int32_t value;
    JS_ToInt32(ctx, &value, argv[0]);

    return JS_NewInt32(ctx, abs(value));
  }

  static JSValue Math_randint(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    int32_t value1;
    JS_ToInt32(ctx, &value1, argv[0]);
    int32_t value2;
    JS_ToInt32(ctx, &value2, argv[1]);

    return JS_NewInt32(ctx, random(value1, value2));
  }

  static JSValue Math_randomBoolean(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    return JS_NewBool(ctx, random(0, 1) == 0);
  }

  static JSValue Math_constrain(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    int32_t value1;
    JS_ToInt32(ctx, &value1, argv[0]);
    int32_t value2;
    JS_ToInt32(ctx, &value2, argv[1]);
    int32_t value3;
    JS_ToInt32(ctx, &value3, argv[2]);

    int32_t ret = value1;
    if( ret < value2 )
      ret = value2;
    if( ret > value3 )
      ret = value3;

    return JS_NewInt32(ctx, ret);
  }

  static JSValue Math_sqrt(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    int32_t value;
    JS_ToInt32(ctx, &value, argv[0]);

    return JS_NewFloat64(ctx, sqrt(value));
  }

  static JSValue Math_sin(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    double value;
    JS_ToFloat64(ctx, &value, argv[0]);

    return JS_NewFloat64(ctx, sin(value));
  }

  static JSValue Math_cos(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    double value;
    JS_ToFloat64(ctx, &value, argv[0]);

    return JS_NewFloat64(ctx, cos(value));
  }

  static JSValue Math_tan(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    double value;
    JS_ToFloat64(ctx, &value, argv[0]);

    return JS_NewFloat64(ctx, tan(value));
  }

  static JSValue pins_digitalWritePin(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    uint32_t value1;
    JS_ToUint32(ctx, &value1, argv[0]);
    uint32_t value2;
    JS_ToUint32(ctx, &value2, argv[1]);

    pinMode(value1, OUTPUT);
    digitalWrite(value1, value2);

    return JS_UNDEFINED;
  }

  static JSValue pins_analogReadPin(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    uint32_t value;
    JS_ToUint32(ctx, &value, argv[0]);

    uint16_t p = analogRead(value);

    return JS_NewUint32(ctx, p);
  }

  static JSValue set_timeout(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    ESP32QuickJS *qjs = (ESP32QuickJS *)JS_GetContextOpaque(ctx);
    uint32_t t;
    JS_ToUint32(ctx, &t, argv[1]);
    uint32_t id =
        qjs->timer.RegisterTimer(JS_DupValue(ctx, argv[0]), millis() + t);
    return JS_NewUint32(ctx, id);
  }

  static JSValue clear_timeout(JSContext *ctx, JSValueConst jsThis, int argc,
                               JSValueConst *argv) {
    ESP32QuickJS *qjs = (ESP32QuickJS *)JS_GetContextOpaque(ctx);
    uint32_t tid;
    JS_ToUint32(ctx, &tid, argv[0]);
    qjs->timer.RemoveTimer(tid);
    return JS_UNDEFINED;
  }

  static JSValue set_interval(JSContext *ctx, JSValueConst jsThis, int argc,
                              JSValueConst *argv) {
    ESP32QuickJS *qjs = (ESP32QuickJS *)JS_GetContextOpaque(ctx);
    uint32_t t;
    JS_ToUint32(ctx, &t, argv[1]);
    uint32_t id =
        qjs->timer.RegisterTimer(JS_DupValue(ctx, argv[0]), millis() + t, t);
    return JS_NewUint32(ctx, id);
  }

  static JSValue esp32_millis(JSContext *ctx, JSValueConst jsThis, int argc,
                              JSValueConst *argv) {
    return JS_NewUint32(ctx, millis());
  }

  static JSValue esp32_gpio_mode(JSContext *ctx, JSValueConst jsThis, int argc,
                                 JSValueConst *argv) {
    uint32_t pin, mode;
    JS_ToUint32(ctx, &pin, argv[0]);
    JS_ToUint32(ctx, &mode, argv[1]);
    pinMode(pin, mode);
    return JS_UNDEFINED;
  }

  static JSValue esp32_deep_sleep(JSContext *ctx, JSValueConst jsThis, int argc,
                                  JSValueConst *argv) {
    uint32_t t;
    JS_ToUint32(ctx, &t, argv[0]);
    ESP.deepSleep(t);  // never return.
    return JS_UNDEFINED;
  }

  static JSValue esp32_restart(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    ESP.restart();
    return JS_UNDEFINED;
  }

  static JSValue esp32_delay(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    uint32_t value;
    JS_ToUint32(ctx, &value, argv[0]);
    delay(value);
    return JS_UNDEFINED;
  }

  static JSValue esp32_set_loop(JSContext *ctx, JSValueConst jsThis, int argc,
                                JSValueConst *argv) {
    ESP32QuickJS *qjs = (ESP32QuickJS *)JS_GetContextOpaque(ctx);
    qjs->setLoopFunc(JS_DupValue(ctx, argv[0]));
    return JS_UNDEFINED;
  }

  static void setText(const char* text, uint8_t rotate = 0){
    uint8_t buffer[2 + 3 * MATRIX_WIDTH * MATRIX_WIDTH] = { 0 };
    buffer[0] = MATRIX_WIDTH;
    buffer[1] = MATRIX_WIDTH;
    char ch = text[0];
    const uint8_t *t = &bmpfont_5x5[(ch - ' ') * MATRIX_WIDTH];
    if( ch >= ' ' && ch <= '~' ){
      for( int y = 0 ; y < MATRIX_WIDTH ; y++ ){
        for( int x = 0 ; x < MATRIX_WIDTH ; x++ ){
          int pos;
          if( rotate == 0 )
            pos = 2 + (y * MATRIX_WIDTH + x) * 3; 
          else if( rotate == 1 )
            pos = 2 + (x * MATRIX_WIDTH + (MATRIX_WIDTH - y - 1)) * 3;
          else if( rotate == 2 )
            pos = 2 + ((MATRIX_WIDTH - y - 1) * MATRIX_WIDTH + (MATRIX_WIDTH - x - 1)) * 3;
          else
            pos = 2 + ((MATRIX_WIDTH - x - 1) * MATRIX_WIDTH + y) * 3;

          // 文字色の設定
          if( t[y] & (0x01 << (MATRIX_WIDTH - x - 1)) ){
            buffer[pos + 0] = (LED_COLOR_ON >> 8) & 0xff;;
            buffer[pos + 1] = (LED_COLOR_ON >> 16) & 0xff;
            buffer[pos + 2] = (LED_COLOR_ON >> 0) & 0xff;
          }
        }
      }
    }

    // LEDマトリクスに表示
    M5.dis.displaybuff(buffer, 0, 0);
  }

#ifdef ENABLE_WIFI
  static JSValue wifi_is_connected(JSContext *ctx, JSValueConst jsThis,
                                   int argc, JSValueConst *argv) {
    return JS_NewBool(ctx, WiFi.status() == WL_CONNECTED);
  }

  static JSValue http_fetch(JSContext *ctx, JSValueConst jsThis, int argc,
                            JSValueConst *argv) {
    ESP32QuickJS *qjs = (ESP32QuickJS *)JS_GetContextOpaque(ctx);
    return qjs->httpFetcher.fetch(ctx, argv[0], argv[1]);
  }
#endif
};
