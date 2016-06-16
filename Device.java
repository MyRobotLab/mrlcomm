package org.myrobotlab.service.interfaces;

/**
 * A device which can be attached to a microcontroller implementers are Sensor
 * and Stepper - perhaps more not sure exactly what all it should impelement -
 * but represents something which can be attached to a microcontroller
 * 
 * This is where all supported devices are defined. They all have a unique type
 * identifier which can be communicated to a microcontroller
 * 
 * It also binds how the microcontroller identifies its service (getIndex())
 * with the service (getName())
 *
 */
public interface Device extends NameProvider {

	/**
	 * The set of defined devices supported. This set is microcontroller
	 * agnostic and new types of devices should be (at least initially) be
	 * defined here. In the future perhaps there will be a registry.
	 * 
	 * The set is grouped into SENSORs and DEVICEs - the distinction being
	 * sensors read and devices write... but there are devices and sensors which
	 * do both - so its not a hard fast rule. The important part is that they
	 * are identified in a global area, and reduce to int values so they are
	 * easily handled by microcontroller code.
	 * 
	 */
	
	// sensors (read)
	public final static int SENSOR_TYPE_ANALOG_PIN_ARRAY = 1;
	public final static int SENSOR_TYPE_DIGITAL_PIN_ARRAY = 2;
	public final static int SENSOR_TYPE_PULSE = 3;
	public final static int SENSOR_TYPE_ULTRASONIC = 4;
	public final static int SENSOR_TYPE_I2C = 5;
	// controllers (write)
	public final static int DEVICE_TYPE_STEPPER = 6;
	public final static int DEVICE_TYPE_MOTOR = 7;
  public final static int DEVICE_TYPE_SERVO = 8;
  public final static int DEVICE_TYPE_I2C = 9;
	/**
	 * index of the device on the micro controller side. an null index would be
	 * a device not yet attached to a microcontroller
	 */

	public Integer getDeviceIndex();

	/**
	 * used by the microcontroller service to set a key index in the peripheral
	 * Device, so that the Device can send this key to the microcontroller to
	 * uniquely identify itself when requesting commands
	 * 
	 * @param index
	 */
	public void setDeviceIndex(Integer index);

	/**
	 * type of device identified by integer so that it can be identified and
	 * handled correctly in the microcontroller code
	 * 
	 * @return type of device
	 */

	public Integer getDeviceType();

	/**
	 * When attaching a Device with a microcontroller the microcontroller code
	 * will need to initialize the new device.
	 * 
	 * These are values which go to the microcontroller code.
	 * 
	 * @return
	 */
	public int[] getDeviceConfig();
}
