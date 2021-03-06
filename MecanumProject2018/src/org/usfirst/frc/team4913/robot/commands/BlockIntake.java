package org.usfirst.frc.team4913.robot.commands;

import static org.usfirst.frc.team4913.robot.OI.joystick;
import static org.usfirst.frc.team4913.robot.OI.xboxController;
import static org.usfirst.frc.team4913.robot.Robot.intaker;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class BlockIntake extends Command {

	private static final double TRIGGER_THRESHOLD = 0.1;

	public BlockIntake() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		super("BlockIntake");
		requires(intaker);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (xboxController.getTriggerAxis(Hand.kLeft) >= TRIGGER_THRESHOLD
				|| xboxController.getTriggerAxis(Hand.kRight) >= TRIGGER_THRESHOLD || joystick.getRawButton(1)) {
			intaker.intakeBlock();
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (xboxController.getTriggerAxis(Hand.kLeft) < TRIGGER_THRESHOLD
				&& xboxController.getTriggerAxis(Hand.kRight) < TRIGGER_THRESHOLD && !joystick.getRawButton(1)) {
			return true;
		}
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		intaker.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
