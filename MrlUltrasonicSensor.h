#ifndef MrlUltrasonicSensor_h
#define MrlUltrasonicSensor_h

// ECHO FINITE STATE MACHINE - NON BLOCKING PULSIN
#define ECHO_STATE_START                    1
#define ECHO_STATE_TRIG_PULSE_BEGIN         2
#define ECHO_STATE_TRIG_PULSE_END           3
#define ECHO_STATE_MIN_PAUSE_PRE_LISTENING  4
#define ECHO_STATE_LISTENING                5
#define ECHO_STATE_GOOD_RANGE               6
#define ECHO_STATE_TIMEOUT                  7

/**
 * Ultrasonic Sensor
 * TODO: add a description about this device, what is it? what does it do?
 * How does it work?
 */
class MrlUltrasonicSensor : public Device {
  public:
    int trigPin;
    int echoPin;
    unsigned long ts;
    unsigned long lastValue;
    unsigned long timeoutUS;
    bool isRanging = false;

    MrlUltrasonicSensor(int deviceId);
    ~MrlUltrasonicSensor();

    void attach(byte trigPin, byte echoPin);
    void update();
    void startRanging(long timeout);
    void stopRanging();
};

#endif
