package frc.robot.input;

/**
 * Class for a bunch of WPI button constants
 */
public enum XboxControllerButtonCode {
    A(1),
	B(2),
	X(3),
	Y(4),
	LB(5),
	RB(6),
	BACK(7),
	START(8),
	L3(9),     //stick button
	R3(10),    //stick button

	//analog triggers
	TRIGGER_LEFT(2),// for left trigger and
	TRIGGER_RIGHT(3),// right trigger

	//analog sticks
	RIGHT_X(4),// for joysticks
	RIGHT_Y(5),
	LEFT_X(0),
	LEFT_Y(1),
	
	//POV Hat //TODO: confirm numbers
	POV_UP(11),
	POV_DOWN(12),
	POV_LEFT(13),
	POV_RIGHT(14)
	;

	public final int value;
	private XboxControllerButtonCode(int initValue) {
		value = initValue;
	}

	public int getCode() {
		return value;
	}
}
