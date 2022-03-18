/**
 * RobotCode.ino
 * 
 * Copyright (c) Vanier Robotics 2022
 * 
 */

#include <CrcLib.h>

// remove comment to enable debug mode (outputs to the serial console, but is significantly slower)
// #define __DEBUG

#define FRONT_LEFT_MOTOR  CRC_PWM_2
#define FRONT_RIGHT_MOTOR CRC_PWM_4
#define BACK_LEFT_MOTOR   CRC_PWM_3
#define BACK_RIGHT_MOTOR  CRC_PWM_1

using namespace Crc;

//////////////////////////////////////////
// Controller

template<size_t AnalogBindingsCount, size_t DigitalBindingsCount>
class Controller
{
public:
  using DigitalBindFunctionPtr = void(*)(bool);
  using AnalogBindFunctionPtr  = void(*)(int8_t);

  struct DigitalBinding
  {
    BUTTON buttonID;

    uint8_t toggleParameters; // bit 1: is toggle, bit 2: is toggled, bit 3: lastValue

    DigitalBindFunctionPtr boundFunction;
  };

  struct AnalogBinding
  {
    ANALOG buttonID;

    AnalogBindFunctionPtr boundFunction;
  };

  bool digitalBind(const BUTTON& buttonID, DigitalBindFunctionPtr functionPtr, const bool& isToggle)
  {
    if (m_currentDigitalBindingsCount >= DigitalBindingsCount)
    {
      #ifdef __DEBUG
      Serial.println("[WARNING] Not enough space in digital bindings.");
      #endif
      
      return false;
    }

    m_digitalBindings[m_currentDigitalBindingsCount] = {buttonID, static_cast<uint8_t>((isToggle & 0x1) | 0x4), functionPtr};
    m_currentDigitalBindingsCount++;

    return true;
  }

  bool analogBind(const ANALOG& buttonID, AnalogBindFunctionPtr functionPtr)
  {
    if (m_currentAnalogBindingsCount >= AnalogBindingsCount)
    {
      #ifdef __DEBUG
      Serial.println("[WARNING] Not enough space in analog bindings.");
      #endif

      return false;
    }

    m_analogBindings[m_currentAnalogBindingsCount] = {buttonID, functionPtr};
    m_currentAnalogBindingsCount++;

    return true;
  }

  void update()
  {
    for (size_t i = 0; i < m_currentAnalogBindingsCount; i++)
    {
      int8_t result = CrcLib::ReadAnalogChannel(m_analogBindings[i].buttonID);
      m_analogBindings[i].boundFunction(result);
    }
    for (size_t i = 0; i < m_currentDigitalBindingsCount; i++)
    {
      bool result = CrcLib::ReadDigitalChannel(m_digitalBindings[i].buttonID);

      if (m_digitalBindings[i].toggleParameters & 0x1)
      {
        // make sure any value other than 0 is true for booleans
        if (result == static_cast<bool>((m_digitalBindings[i].toggleParameters & 0x4)))
        {
          continue;
        }

        if (result)
        {
          // toggle the value
          m_digitalBindings[i].toggleParameters ^= 0x2; // switch second bit
          m_digitalBindings[i].boundFunction(m_digitalBindings[i].toggleParameters & 0x2);
        }

        m_digitalBindings[i].toggleParameters ^= 0x4;
      }
      else
      {
        m_digitalBindings[i].boundFunction(result);
      }

    }
  }

private:
  DigitalBinding m_digitalBindings[DigitalBindingsCount];
  size_t         m_currentDigitalBindingsCount = 0;
  AnalogBinding  m_analogBindings[AnalogBindingsCount];
  size_t         m_currentAnalogBindingsCount = 0;
};

//////////////////////////////////////////

//////////////////////////////////////////
// Drive Modes

class ModeManager;

class Mode
{
public:
  static ModeManager& modeManager;

  virtual void update(float dt) = 0;

  virtual void load()   = 0;
  virtual void unload() = 0;
};

class ModeManager
{
public:
  void changeMode(Mode* nextMode)
  {
    m_nextMode = nextMode;
  }

  void update(float dt)
  {
    if (m_currentMode)
      m_currentMode->update(dt);

    if (m_nextMode)
    {
      if (m_currentMode)
        m_currentMode->unload();

      m_currentMode = m_nextMode;
      m_currentMode->load();
      m_nextMode = nullptr;
    }
  }

private:
  Mode* m_currentMode = nullptr;
  Mode* m_nextMode    = nullptr;
};

class IdleMode: public Mode
{
public:
  static Mode* nextMode;

  IdleMode()
  {
    m_bindings.digitalBind(BUTTON::START, start, false);
  }

  static void start(bool pressed)
  {
    if (pressed)
    {
      Mode::modeManager.changeMode(nextMode);
    }
  }

  void update(float dt) override
  {
    m_bindings.update();
  }

  void load() override
  {
    #ifdef __DEBUG
    Serial.println("Idle Mode. Press START on Controller.");
    #endif
  }

  void unload() override
  {
  }

private:
  Controller<1, 1> m_bindings; // we have one analog input to avoid having a zero-sized array
};

class TankMode : public Mode
{
public:
  static Mode* mecanumMode;

  TankMode()
  {
    m_bindings.digitalBind(BUTTON::COLORS_UP,    [](bool)         { Mode::modeManager.changeMode(mecanumMode); }, true);
    m_bindings.digitalBind(BUTTON::COLORS_LEFT,  [](bool pressed) { outputChain = pressed; },                     false);
    m_bindings.digitalBind(BUTTON::COLORS_RIGHT, [](bool pressed) { boost = pressed; },                           false);
    m_bindings.digitalBind(BUTTON::COLORS_LOW,   [](bool toggled) { inputActivated = pressed; },                  true);
    m_bindings.digitalBind(BUTTON::ARROW_DOWN,   [](bool pressed) { if (pressed) changeAnglePreset(false); },     false);
    m_bindings.digitalBind(BUTTON::ARROW_UP,     [](bool pressed) { if (pressed) changeAnglePreset(true) },       false);
    
    m_bindings.analogBind(ANALOG::JOYSTICK1_X, [](int8_t value) { direction = value; });
    m_bindings.analogBind(ANALOG::GACHETTE_R,  [](int8_t value) { forward += (static_cast<int>(value) + 128) / 2; }); // /4 + inverse?
    m_bindings.analogBind(ANALOG::GACHETTE_L,  [](int8_t value) { forward -= (static_cast<int>(value) + 128) / 2; });
    m_bindings.analogBind(ANALOG::JOYSTICK2_Y, [](int8_t value) { if (value <= -32) slideAngleChange = 5; else if (value >= 5) slideAngleChange = -5; });
  }

  void update(float dt) override
  {
    m_bindings.update();
    const float result = static_cast<float>(slideAngle) + static_cast<float>(slideAngleChange) * dt;
    if (result > 112)
    {
      slideAngle = 112;
    }
    else if (result < -64)
    {
      slideAngle = -64;
    }
    else
    {
      slideAngle = result;
    }

    float m = 1; // [0.5, 2]
    float right = max(min((- m * static_cast<float>(direction) + static_cast<float>(forward)) * (boost ? 1 : 0.5), 127), -128);
    float left = max(min((m * static_cast<float>(direction) + static_cast<float>(forward)) * (boost ? 1 : 0.5), 127), -128);

    #ifdef __DEBUG
    Serial.print("[TANK] Boost: "); Serial.print(boost);
    Serial.print(" Chain: ");       Serial.print(outputChain);
    Serial.print(" Forward: ");     Serial.print(static_cast<int>(forward));
    Serial.print(" Direction: ");   Serial.print(static_cast<int>(direction));
    Serial.print(" Slide Angle: "); Serial.print(static_cast<int>(slideAngle));
    Serial.print(" Right: ");       Serial.print(static_cast<int>(right));
    Serial.print(" Left: ");        Serial.println(static_cast<int>(left));
    #endif
  
    CrcLib::MoveTank(left, right, FRONT_LEFT_MOTOR, BACK_LEFT_MOTOR, FRONT_RIGHT_MOTOR, BACK_RIGHT_MOTOR);
    
    forward          = 0;
    slideAngleChange = 0;
  }

  void load() override
  {
  }

  void unload() override
  {
  }

  static void changeSlideAnglePreset(bool goingUp)
  {
    slideAngle = (goingUp ? 10 : -10);
  }

private:
  Controller<4, 6> m_bindings;


  static int8_t forward;
  static int8_t direction;
  static bool   boost;
  static bool   inputActivated;
  static float  slideAngle;
  static int8_t slideAngleChange;
  static bool   outputChain;
};

int8_t TankMode::forward          = 0;
int8_t TankMode::direction        = 0;
bool   TankMode::boost            = false;
bool   TankMode::inputActivated   = false;
float  TankMode::slideAngle       = 0;
int8_t TankMode::slideAngleChange = 0;
bool   TankMode::outputChain      = false;

class MecanumMode : public Mode
{
public:
  static Mode* tankMode;

  MechanumMode()
  {
    m_bindings.digitalBind(BUTTON::COLORS_UP,   [](bool)         { Mode::modeManager.changeMode(tankMode); },  true);
    m_bindings.digitalBind(BUTTON::COLORS_LEFT, [](bool pressed) { outputChain = pressed; },                   false);
    m_bindings.digitalBind(BUTTON::COLORS_LOW,  [](bool toggled) { inputActivated = pressed; },                true);
    m_bindings.digitalBind(BUTTON::ARROW_DOWN,  [](bool pressed) { if (pressed) changeAnglePreset(false); },   false);
    m_bindings.digitalBind(BUTTON::ARROW_UP,    [](bool pressed) { if (pressed) changeAnglePreset(true) },     false);
    m_bindings.digitalBind(BUTTON::L1,          [](bool pressed) { if (pressed) MechanumMode::strafe -= 64; }, false);
    m_bindings.digitalBind(BUTTON::R1,          [](bool pressed) { if (pressed) MechanumMode::strafe += 64; }, false);
    
    m_bindings.analogBind(ANALOG::JOYSTICK1_X, [](int8_t value) { MechanumMode::yaw = value; });
    m_bindings.analogBind(ANALOG::JOYSTICK1_Y, [](int8_t value) { MechanumMode::forward = -value; });
    m_bindings.analogBind(ANALOG::JOYSTICK2_Y, [](int8_t value) { if (value <= -32) MechanumMode::slideAngleChange = 5; else if (value >= 32) MechanumMode::slideAngleChange = -5; });
  }

  void update(float dt) override
  {
    m_bindings.update();
    const float result = static_cast<float>(slideAngle) + static_cast<float>(slideAngleChange) * dt;
    if (result > 112)
    {
      slideAngle = 112;
    }
    else if (result < -64)
    {
      slideAngle = -64;
    }
    else
    {
      slideAngle = result;
    }

    #ifdef __DEBUG
    Serial.print("[MECHANUM] Forward: "); Serial.print(static_cast<int>(forward));
    Serial.print(" Yaw: ");               Serial.print(static_cast<int>(yaw));
    Serial.print(" Strafe: ");            Serial.print(static_cast<int>(strafe));
    Serial.print(" Chain: ");             Serial.print(outputChain);
    Serial.print(" Slide Direction: ");   Serial.println(static_cast<int>(slideAngle));
    #endif

    CrcLib::MoveHolonomic(forward, yaw, strafe,FRONT_LEFT_MOTOR, BACK_LEFT_MOTOR, FRONT_RIGHT_MOTOR, BACK_RIGHT_MOTOR);

    strafe           = 0;
    slideAngleChange = 0;
  }

  void load() override
  {
  }

  void unload() override
  {
  }

  static void changeSlideAnglePreset(bool goingUp)
  {
    slideAngle = (goingUp ? 10 : -10);
  }

private:
  Controller<3, 6> m_bindings;

  static int8_t forward;
  static int8_t yaw;
  static int8_t strafe;
  static bool   inputActivated;
  static float  slideAngle;
  static int8_t slideAngleChange;
  static bool   outputChain;
};

int8_t MecanumMode::forward          = 0;
int8_t MecanumMode::yaw              = 0;
int8_t MecanumMode::strafe           = 0;
bool   MecanumMode::inputActivated   = false;
float  MecanumMode::slideAngle       = 0;
int8_t MecanumMode::slideAngleChange = 0;
bool   MecanumMode::outputChain      = false;

//////////////////////////////////////////

//////////////////////////////////////////
// robot code

ModeManager driveModeManager;
IdleMode    idleDriveMode;
TankMode    tankDriveMode;
MecanumMode mecanumDriveMode;

ModeManager& Mode::modeManager     = driveModeManager;
Mode*        IdleMode::nextMode    = &tankDriveMode;
Mode*        TankMode::mecanumMode = &mecanumDriveMode;
Mode*        MecanumMode::tankMode = &tankDriveMode;

void setup()
{
  CrcLib::Initialize();

  #ifdef __DEBUG
  Serial.begin(9600);
  #endif

  CrcLib::InitializePwmOutput(BACK_LEFT_MOTOR);
  CrcLib::InitializePwmOutput(FRONT_LEFT_MOTOR, true);
  CrcLib::InitializePwmOutput(BACK_RIGHT_MOTOR, true);
  CrcLib::InitializePwmOutput(FRONT_RIGHT_MOTOR, true);

  driveModeManager.changeMode(&idleDriveMode);
}

void loop()
{
  CrcLib::Update();

  if (CrcLib::IsCommValid())
  {
    // TODO: Add a global clock to keep track of time
    driveModeManager.update(0.033333f);
  }
  else
  {
    #ifdef __DEBUG
    Serial.println("Controller Not Connected.");
    #endif
  }
}

//////////////////////////////////////////
