// Spatial Media Lab - tracker
//
// © Daniel Rudrich 2020
// © Kay Sievers 2020

#include <V2BNO055.h>
#include <V2Buttons.h>
#include <V2Color.h>
#include <V2Device.h>
#include <V2LED.h>
#include <V2MIDI.h>
#include <Wire.h>

V2DEVICE_METADATA("org.spatialmedialab.tracker", 9, "spatialmedialab:samd:tracker");

static V2LED LED(2, PIN_LED_WS2812, &sercom2, SPI_PAD_0_SCK_1, PIO_SERCOM);

struct TaitBryan {
    float yaw;
    float pitch;
    float roll;    
};

class Quaternion {
public:
  float w{1};
  float x{0};
  float y{0};
  float z{0};

  constexpr Quaternion() = default;
  constexpr Quaternion(float qw, float qx, float qy, float qz) : w(qw), x(qx), y(qy), z(qz) {}

  float getLength() {
    return sqrtf(w * w + x * x + y * y + z * z);
  }

  void normalize() {
    const float l = getLength();
    if (fabs(l) > 0.000001f)
      scale(1.f / l);
  }

  void conjugate() {
    x = -x;
    y = -y;
    z = -z;
  }

  Quaternion getConjugate() {
    return Quaternion(w, -x, -y, -z);
  }

  Quaternion operator*(const Quaternion &q) {
    return Quaternion(w * q.w - x * q.x - y * q.y - z * q.z,
                      w * q.x + x * q.w + y * q.z - z * q.y,
                      w * q.y - x * q.z + y * q.w + z * q.x,
                      w * q.z + x * q.y - y * q.x + z * q.w);
  }

  void scale(float factor) {
    w *= factor;
    x *= factor;
    y *= factor;
    z *= factor;
  }

  TaitBryan toTaitBryanAngles() const {
    const float ysqr = y * y;
    
    float t0 = 2.0f * (w * z + x * y);
    float t1 = 1.0f - 2.0f * (ysqr + z * z);
    const float yaw = std::atan2 (t0, t1);
    
    t0 = 2.0f * (w * y - z * x);
    t0 = t0 > 1.0f ? 1.0f : t0;
    t0 = t0 < -1.0f ? -1.0f : t0;
    const float pitch = std::asin (t0);
    
    t0 = 2.0f * (w * x + y * z);
    t1 = 1.0f - 2.0f * (x * x + ysqr);
    const float roll = std::atan2 (t0, t1);

    return {yaw, pitch, roll};
  }
};

class Vector {
public:
  float x{1};
  float y{0};
  float z{0};

  constexpr Vector() = default;
  constexpr Vector(float vx, float vy, float vz) : x(vx), y(vy), z(vz) {}

  void normalize() {
    const float l = getLength();
    if (fabs(l) > 0.000001f)
      scale(1.f / l);
  }

  float getLength() {
    return sqrtf(x * x + y * y + z * z);
  }

  void scale(float factor) {
    x *= factor;
    y *= factor;
    z *= factor;
  }

  Vector cross(const Vector other) {
    return Vector(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
  }
};

static class Sensor : public V2BNO055 {
public:
  Sensor() : V2BNO055(&Wire, PIN_SENSOR_RESET, PIN_SENSOR_INTERRUPT) {}

  Quaternion getCorrectedOrientation() {
    _raw                      = getOrientation();
    const Quaternion steering = _idleOrientationConj * _raw;
    _currentOrientation       = _calibrationConj * steering * _calibration;
    return _currentOrientation;
  }

  void resetOrientation() {
    _idleOrientationConj = _raw.getConjugate();
  }

  void startCalibration() {
    _idleGravity = getGravity();
    resetOrientation();
  }

  void finishCalibration() {
    Vector z = _idleGravity;
    z.scale(-1);
    z.normalize();

    Vector y = getGravity().cross(_idleGravity);
    y.normalize();

    Vector x = y.cross(z);
    z.normalize();

    const float w = 0.5f * sqrtf(1.f + x.x + y.y + z.z);
    const float f = 1.f / (4.f * w);
    setCalibration(Quaternion(w, f * (y.z - z.y), f * (z.x - x.z), f * (x.y - y.x)));
  }

  Quaternion getCalibration() {
    return _calibration;
  }

  void setCalibration(Quaternion calibration) {
    _calibration     = calibration;
    _calibrationConj = calibration.getConjugate();
  }

private:
  Quaternion _idleOrientationConj{};
  Vector _idleGravity{};
  Quaternion _raw{};
  Quaternion _calibration{};
  Quaternion _calibrationConj{};
  Quaternion _currentOrientation{};

  Quaternion getOrientation() {
    float w, x, y, z;
    readQuaternion(w, x, y, z);
    return Quaternion(w, x, y, z);
  }

  Vector getGravity() {
    float x, y, z;
    readGravity(x, y, z);
    Vector gravity(-x, -y, -z);
    gravity.normalize();
    return gravity;
  }
} Sensor;

static class Device : public V2Device {
public:
  Device() : V2Device() {
    metadata.vendor      = "Spatial Media Lab";
    metadata.product     = "tracker";
    metadata.description = "Orientation Sensor";
    metadata.home        = "https://spatialmedialab.org/#tracker/";

    system.ports.announce = 0;
    system.download       = "https://spatial-media-lab.github.io/download/";

    configuration = {.magic{0x9e020000 | usb.pid}, .size{sizeof(config)}, .data{&config}};
  }

  // Config, written to EEPROM
  struct {
    uint8_t channel;
    struct {
      float w, x, y, z;
    } calibration;
  } config{};


   void setLEDIdle()
  {
    LED.setHSV(0, V2Color::Blue, 1, 0.5);
    LED.setHSV(1, V2Color::Red, 1, 0.5);    
  }

  void reset() {
    digitalWrite(PIN_LED_ONBOARD, LOW);
    LED.reset();

    setLEDIdle();
  }

  void allNotesOff() {
    // Send all current values when 'Stop' is pressed in the Audio Workstation.
    _hires.reset();
    sendControls();
  }

  void loop() {
    if ((unsigned long)(micros() - _usec) < 10 * 1000) // 10 * 1000 -> 100Hz
      return;

    sendControls();
    _usec = micros();
  }

private:
  V2MIDI::CC::HighResolution<V2MIDI::CC::GeneralPurpose1, 4> _hires;
  long _usec{};

  void sendControls() {
    const Quaternion q = Sensor.getCorrectedOrientation();
    if (_hires.set(V2MIDI::CC::GeneralPurpose1 + 0, (q.w + 1.f) * 8191.f))
      _hires.send(this, &usb.midi, config.channel, V2MIDI::CC::GeneralPurpose1 + 0);

    if (_hires.set(V2MIDI::CC::GeneralPurpose1 + 1, (q.x + 1.f) * 8191.f))
      _hires.send(this, &usb.midi, config.channel, V2MIDI::CC::GeneralPurpose1 + 1);

    if (_hires.set(V2MIDI::CC::GeneralPurpose1 + 2, (q.y + 1.f) * 8191.f))
      _hires.send(this, &usb.midi, config.channel, V2MIDI::CC::GeneralPurpose1 + 2);

    if (_hires.set(V2MIDI::CC::GeneralPurpose1 + 3, (q.z + 1.f) * 8191.f))
      _hires.send(this, &usb.midi, config.channel, V2MIDI::CC::GeneralPurpose1 + 3);

    const TaitBryan t = q.toTaitBryanAngles();

    if (_hires.set(V2MIDI::CC::GeneralPurpose1 + 4, (t.yaw / 1.570796327f + 1.f) * 8191.f))
      _hires.send(this, &usb.midi, config.channel, V2MIDI::CC::GeneralPurpose1 + 4);

    if (_hires.set(V2MIDI::CC::GeneralPurpose1 + 5, (t.pitch / 1.570796327f + 1.f) * 8191.f))
      _hires.send(this, &usb.midi, config.channel, V2MIDI::CC::GeneralPurpose1 + 5);

    if (_hires.set(V2MIDI::CC::GeneralPurpose1 + 6, (t.roll / 1.570796327f + 1.f) * 8191.f))
      _hires.send(this, &usb.midi, config.channel, V2MIDI::CC::GeneralPurpose1 + 6);
  }

  void handleControlChange(uint8_t channel, uint8_t controller, uint8_t value) override {
    switch (controller) {
      case V2MIDI::CC::AllSoundOff:
      case V2MIDI::CC::AllNotesOff:
        allNotesOff();
        break;
    }
  }

  void handleSystemReset() override {
    reset();
  }

  void exportOutput(JsonObject json) override {
    json["channel"] = config.channel;

    JsonArray json_controllers      = json.createNestedArray("controllers");
    JsonObject json_orientation_w   = json_controllers.createNestedObject();
    json_orientation_w["name"]      = "Quaternion W";
    json_orientation_w["number"]    = V2MIDI::CC::GeneralPurpose1 + 0;
    json_orientation_w["value"]     = _hires.getMSB(V2MIDI::CC::GeneralPurpose1 + 0);
    json_orientation_w["valueFine"] = _hires.getLSB(V2MIDI::CC::GeneralPurpose1 + 0);

    JsonObject json_orientation_x   = json_controllers.createNestedObject();
    json_orientation_x["name"]      = "Quaternion X";
    json_orientation_x["number"]    = V2MIDI::CC::GeneralPurpose1 + 1;
    json_orientation_x["value"]     = _hires.getMSB(V2MIDI::CC::GeneralPurpose1 + 1);
    json_orientation_x["valueFine"] = _hires.getLSB(V2MIDI::CC::GeneralPurpose1 + 1);

    JsonObject json_orientation_y   = json_controllers.createNestedObject();
    json_orientation_y["name"]      = "Quaternion Y";
    json_orientation_y["number"]    = V2MIDI::CC::GeneralPurpose1 + 2;
    json_orientation_y["value"]     = _hires.getMSB(V2MIDI::CC::GeneralPurpose1 + 2);
    json_orientation_y["valueFine"] = _hires.getLSB(V2MIDI::CC::GeneralPurpose1 + 2);

    JsonObject json_orientation_z   = json_controllers.createNestedObject();
    json_orientation_z["name"]      = "Quaternion Z";
    json_orientation_z["number"]    = V2MIDI::CC::GeneralPurpose1 + 3;
    json_orientation_z["value"]     = _hires.getMSB(V2MIDI::CC::GeneralPurpose1 + 3);
    json_orientation_z["valueFine"] = _hires.getLSB(V2MIDI::CC::GeneralPurpose1 + 3);

    JsonObject json_orientation_yaw   = json_controllers.createNestedObject();
    json_orientation_yaw["name"]      = "Yaw";
    json_orientation_yaw["number"]    = V2MIDI::CC::GeneralPurpose1 + 4;
    json_orientation_yaw["value"]     = _hires.getMSB(V2MIDI::CC::GeneralPurpose1 + 4);
    json_orientation_yaw["valueFine"] = _hires.getLSB(V2MIDI::CC::GeneralPurpose1 + 4);

    JsonObject json_orientation_pitch   = json_controllers.createNestedObject();
    json_orientation_pitch["name"]      = "Pitch";
    json_orientation_pitch["number"]    = V2MIDI::CC::GeneralPurpose1 + 5;
    json_orientation_pitch["value"]     = _hires.getMSB(V2MIDI::CC::GeneralPurpose1 + 5);
    json_orientation_pitch["valueFine"] = _hires.getLSB(V2MIDI::CC::GeneralPurpose1 + 5);

    JsonObject json_orientation_roll   = json_controllers.createNestedObject();
    json_orientation_roll["name"]      = "Roll";
    json_orientation_roll["number"]    = V2MIDI::CC::GeneralPurpose1 + 6;
    json_orientation_roll["value"]     = _hires.getMSB(V2MIDI::CC::GeneralPurpose1 + 6);
    json_orientation_roll["valueFine"] = _hires.getLSB(V2MIDI::CC::GeneralPurpose1 + 6);
  }

  void importConfiguration(JsonObject json) override {
    if (!json["channel"].isNull()) {
      uint8_t channel = json["channel"];
      if (channel < 1)
        config.channel = 0;
      else if (channel > 16)
        config.channel = 15;
      else
        config.channel = channel - 1;
    }
  }

  void exportConfiguration(JsonObject json) override {
    json["#channel"] = "The MIDI channel to send control values and notes";
    json["channel"]  = config.channel + 1;
  }

  void exportSystem(JsonObject json) override {
    json["temperature"] = truncf(Sensor.readTemperature() * 10.f) / 10.f;
    json["calibW"] = truncf(config.calibration.w * 1000.0f) / 1000.0f;
    json["calibX"] = truncf(config.calibration.x * 1000.0f) / 1000.0f;
    json["calibY"] = truncf(config.calibration.y * 1000.0f) / 1000.0f;
    json["calibZ"] = truncf(config.calibration.z * 1000.0f) / 1000.0f;
  }
} Device;

// Dispatch MIDI packets
static class MIDI {
public:
  void loop() {
    if (!Device.usb.midi.receive(&_midi))
      return;

    if (_midi.getPort() == 0)
      Device.dispatch(&Device.usb.midi, &_midi);
  }

private:
  V2MIDI::Packet _midi{};
} MIDI;

static class Button : public V2Buttons::Button {
public:
  Button() : V2Buttons::Button(&_config, PIN_BUTTON) {}

private:
  const V2Buttons::Config _config{.click_usec{150 * 1000}, .hold_usec{300 * 1000}};

  bool isRainbow = false;
  
  void handleClick(uint8_t count) override {
    if (count == 0)
    {
        LED.splashHSV(0.8, V2Color::Red, 1, 0.5);
        Sensor.resetOrientation();
    } 
    else
    {
        if (isRainbow)
        {
            LED.rainbow(0, 1.0f);
            isRainbow = false;
        }
        else 
        {
            LED.rainbow(1, 1.0f);
            isRainbow = true;
        }
    }
    
  }

  void handleHold(uint8_t count) override {
    LED.setHSV(V2Color::Yellow, 1, 0.5);
    Sensor.startCalibration();
  }

  void handleRelease() override {
    Device.setLEDIdle();
    LED.splashHSV(1.0, 2, V2Color::Green, 1, 0.5);
    Sensor.finishCalibration();

    auto calibration = Sensor.getCalibration();
    Device.config.calibration.w = calibration.w;
    Device.config.calibration.x = calibration.x;
    Device.config.calibration.y = calibration.y;
    Device.config.calibration.z = calibration.z;
    Device.writeConfiguration(&Device.config, sizeof(Device.config));
  }
} Button;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(1);

  static Adafruit_USBD_WebUSB WebUSB;
  static WEBUSB_URL_DEF(WEBUSBLandingPage, 1 /*https*/, "spatial-media-lab.github.io/configure/");
  WebUSB.begin();
  WebUSB.setLandingPage(&WEBUSBLandingPage);

  LED.begin();
  LED.setMaxBrightness(0.5);
  Button.begin();
  Device.begin();

  // Uses delay(), needs to be after Device.begin();
  Sensor.begin();
  Device.reset();

  Sensor.setCalibration(Quaternion{
        Device.config.calibration.w,
        Device.config.calibration.x,
        Device.config.calibration.y,
        Device.config.calibration.z});
}

void loop() {
  LED.loop();
  MIDI.loop();
  V2Buttons::loop();
  Device.loop();

  if (Device.idle())
    Device.sleep();
}
