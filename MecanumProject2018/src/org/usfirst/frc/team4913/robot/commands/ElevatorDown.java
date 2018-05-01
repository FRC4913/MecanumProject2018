package org.usfirst.frc.team4913.robot.commands;

import static org.usfirst.frc.team4913.robot.Robot.elevator;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorDown extends Command {

	public ElevatorDown() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		super("ElevatorDown");
		requires(elevator);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		elevator.down();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return elevator.isDownLimitSet();
	}

	// Called once after isFinished returns true
	protected void end() {
		elevator.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
