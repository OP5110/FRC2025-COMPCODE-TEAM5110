// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntake;
import frc.robot.subsystems.AlgaeAngle.AlgaeAngleSubsystem;
import frc.robot.subsystems.CoralIntake.CoralIntake;
// import frc.robot.subsystems.Intake.IntakeSubsystem;
//import frc.robot.subsystems.AlgaeAngle.AlgaeAngleSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
//public class BasicAuto extends SequentialCommandGroup {
  /** Creates a new BasicAuto. */



  public final class BasicAuto {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    
    public static Command arm_test(
      SwerveSubsystem drivebase,
      CoralIntake coralSubsystem, 
      AlgaeIntake intakeSubsystem, 
      AlgaeAngleSubsystem armSubsystem) {
      return Commands.sequence(
  
      // arm down
      armSubsystem.autoAlgaeDown(),
      new WaitCommand(3),
      armSubsystem.stopAlgae()
  
          );   
       }


//   public static Command shootNote_pos3(
//     SwerveSubsystem drivebase,
//     ShooterSubsystem shooterSubsystem,
//     // IntakeSubsystem intakeSubsystem, 
//     ConvSubsystem armSubsystem) 
//     {
//     return Commands.sequence(
  
//     // arm down
//     armSubsystem.autoArmDown(),
//     new WaitCommand(3.5),
//     armSubsystem.stopArm(),

//     // run shooter
//     shooterSubsystem.runShooter(-1),
//     new WaitCommand(1.0),
//     //intakeSubsystem.runIntake(-1),
//     new WaitCommand(1),
//     // intakeSubsystem.stopIntake(),
//     shooterSubsystem.stopShooter(),

//     //new WaitCommand(1),
//    // drivebase.getAutonomousCommand("Middle Out"),

//     new WaitCommand(1)

//     //new InstantCommand(()->drivebase.stopSwerve())
//     );   
//   }

  }


