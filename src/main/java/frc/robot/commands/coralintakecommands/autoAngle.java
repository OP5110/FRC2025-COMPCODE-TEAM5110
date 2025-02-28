// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralintakecommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.CoralIntake.CoralIntake;

public class autoAngle extends Command {
  /** Creates a new autoArm. */
  CoralIntake m_CoralSubsystem;
  Timer timer;

  public autoAngle(CoralIntake CoralSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_CoralSubsystem = CoralSubsystem;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_CoralSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_CoralSubsystem.runCoral(.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_CoralSubsystem.stopCoral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() > 2.5) {
      return true; //causes the command to end if the timer is greater than 2 seconds
    }
    else {
      return false; //if the timer is not greater than 2 seconds, the command will not end
    }
     //when thes value is true, the scheduler runs the command once and then ends the command
  }
}
