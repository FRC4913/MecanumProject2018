package org.usfirst.frc.team4913.robot.commands;

import static org.usfirst.frc.team4913.robot.Robot.climber;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotUp extends Command {

	public RobotUp() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		super("RobotUp");
		requires(climber);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		climber.robotUp();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		climber.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
