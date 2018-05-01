package org.usfirst.frc.team4913.robot.subsystems;

import org.usfirst.frc.team4913.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Elevator extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public static double ELEVATOR_UP_SPEED = 1.0;
	public static double ELEVATOR_DOWN_SPEED = -1.0;

	Spark elevatorMotor = new Spark(RobotMap.ELEVATOR_MOTOR_PORT);
	DigitalInput switchUp = new DigitalInput(RobotMap.SWITCH_UP_PORT);
	DigitalInput switchDown = new DigitalInput(RobotMap.SWITCH_DOWN_PORT);

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public void up() {
		elevatorMotor.set(ELEVATOR_UP_SPEED);
		SmartDashboard.putBoolean("uplimit", switchUp.get());
		SmartDashboard.putBoolean("downlimit", switchDown.get());
	}

	public void down() {
		elevatorMotor.set(ELEVATOR_DOWN_SPEED);
		SmartDashboard.putBoolean("uplimit", switchUp.get());
		SmartDashboard.putBoolean("downlimit", switchDown.get());
	}

	public boolean isUpLimitSet() {
		return !switchUp.get();
	}

	public boolean isDownLimitSet() {
		return !switchDown.get();
	}

	public void stop() {
		elevatorMotor.set(0);
	}
}
