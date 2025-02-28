// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PowerDistribution;
//import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OperatorConstants;

//import frc.robot.commands.CoralCommands.autoCoral;
//import frc.robot.commands.swervedrive.auto.BasicAuto;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.AlgaeAngle.AlgaeAngleSubsystem;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntake;

import frc.robot.subsystems.Puller.Puller;

import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.commands.elevatorcommands.positionElevator;

import edu.wpi.first.wpilibj.DigitalInput;

public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */


  
  private final Puller m_puller = new Puller();
 
  private final AlgaeAngleSubsystem m_angle = new AlgaeAngleSubsystem();
  private final AlgaeIntake m_algaeIntake = new AlgaeIntake();
  
  
  private final CoralIntake m_CoralIntake = new CoralIntake();
  private final Elevator m_elevator = new Elevator();

  private final DigitalInput convLimitSwitch = new DigitalInput(0);


  // The robot's subsystems and commands are defined here...
  XboxController driverXbox2 = new XboxController(1);

// private final Command m_autoShootnote = BasicAuto.shootNote_pos3(drivebase, m_shooter, m_intake, m_conv);




  private void configureBindings()
  {

   
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    //Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);

    //Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

    //Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    //Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    // if (RobotBase.isSimulation())
    // {
    //   drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    // } else
    // {
    //   drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // }

    // if (Robot.isSimulation())
    // {
    //   driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    //   driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    // }
    // if (DriverStation.isTest())
    // {
    //   drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

    //   driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //   driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
    //   driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //   driverXbox.back().whileTrue(drivebase.centerModulesCommand());
    //   driverXbox.leftBumper().onTrue(Commands.none());
    //   driverXbox.rightBumper().onTrue(Commands.none());
    // } else
    // {
    //   driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //   driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    //   driverXbox.b().whileTrue(
    //       drivebase.driveToPose(
    //           new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           );
    //   driverXbox.start().whileTrue(Commands.none());
    //   driverXbox.back().whileTrue(Commands.none());
      
    //   driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //   driverXbox.rightBumper().onTrue(Commands.none());
    // }

    // // Climber, DPad
    //Constants.operatorController.povUp().onTrue(m_climber.setHeight(ClimberSubsystem.ClimberState.EXTENDED.height));
    //Constants.operatorController.povDown().onTrue(m_climber.setHeight(ClimberSubsystem.ClimberState.RETRACTED.height));
    
    // Constants.operatorController.rightBumper().whileTrue(m_climberR.uhOhNoWorky(-.75)).whileFalse(m_climberR.uhOhNoWorkyStop());
    // Constants.operatorController.rightTrigger().whileTrue(m_climberR.uhOhNoWorky(.75)).whileFalse(m_climberR.uhOhNoWorkyStop());
    
    // Constants.operatorController.leftTrigger().whileTrue(m_climberL.uhOhNoWorky2(-.75)).whileFalse(m_climberL.uhOhNoWorky2Stop());
    // Constants.operatorController.leftBumper().whileTrue(m_climberL.uhOhNoWorky2(.75)).whileFalse(m_climberL.uhOhNoWorky2Stop());


    // Conv, Bumpers and run when inake
    //Constants.operatorController.axisGreaterThan(5, 0.1).whileTrue(m_conv.runConvIntake()); // Looks w'rong because of Intake stuff but we just running the Conv.
    //Constants.operatorController.axisLessThan(5, 0.1).whileTrue(m_conv.stopConv());

    //++++++++++++++++++++++ Start 2025 Key Bindings +++++++++++++++++++++
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



    //the button used to be pov left
    Constants.operatorController.axisGreaterThan(1, 0.1)
    .onTrue(m_angle.runAlgae(0.3))
    .onFalse(m_angle.runAlgae(0.03));
    
    //the button used to be pov right
    Constants.operatorController.axisLessThan(1, -0.1)
    .whileTrue(Commands.run(() -> {
        if (convLimitSwitch.get()) { // Limit switch NOT pressed (returns true)
            m_angle.runAlgae(-0.3).schedule();
        }
    }, m_angle))
    .onFalse(m_angle.runAlgae(.03));


// Constants.operatorController
//     .axisGreaterThan(1, 0.05) // Prevent small drift from activating
//     .or(Constants.operatorController.axisLessThan(1, -0.05))
//     .whileTrue(Commands.run(() -> {
//         double stickValue = -Constants.operatorController.getRightY(); // Invert if needed

//         // Apply cubic scaling for smoother control while keeping direction
//         double speed = Math.pow(stickValue, 3);

//         // Ensure a minimum power threshold for movement
//         if (Math.abs(stickValue) > 0.05) {
//             if (speed > 0.3) speed = 0.3;  // Limit max upward speed
//             else if (speed < -0.1) speed = -0.1;  // Limit max downward speed
//             else if (Math.abs(speed) < 0.07) speed = 0.07 * Math.signum(speed);  // Ensure minimum movement power
//         } else {
//             speed = 0; // Prevent small oscillations near center
//         }

//         m_elevator.runElevator(speed).schedule();
//     }, m_elevator))
//     .onFalse(m_elevator.runElevator(0.02));




    Constants.operatorController.axisGreaterThan(5, -0.1) // Assuming axis 1 is right stick Y
    .onTrue(m_elevator.runElevator(0.3))
    .onFalse(m_elevator.runElevator(0.02));

    Constants.operatorController.axisLessThan(5, 0.1)
    .onTrue(m_elevator.runElevator(-0.2))
    .onFalse(m_elevator.runElevator(0.02));

// LIMIT SWITCH TEST CODE BELOW
    // Constants.operatorController.povDown()
    // .whileTrue(Commands.run(() -> {
    //     if (convLimitSwitch.get()) { // Limit switch NOT pressed (returns true)
    //         m_elevator.runElevator(-0.1).schedule();
    //     }
    // }, m_elevator))
    // .onFalse(m_elevator.runElevator(.02));

    // Elevator Positioning
    Constants.operatorController.a().whileTrue(m_elevator.moveToCoralLowBranch());

    Constants.operatorController.b().whileTrue(m_elevator.moveToCoralMidBranch());

    Constants.operatorController.x().whileTrue(m_elevator.moveToCoralHighBranch());    

    //Constants.operatorController.y().whileTrue(m_elevator.moveToCoralBaseTrough());

     // Constants.operatorController.start().onTrue(new autoShoot(m_shooter));
    Constants.operatorController.rightTrigger(.5).whileTrue(m_CoralIntake.runCoral(.3))
     .whileFalse(m_CoralIntake.runCoral(0));

    // Coral, Auto Shoot
    Constants.operatorController.back().whileTrue(m_CoralIntake.reverseCoral());

    Constants.operatorController.start().whileTrue(m_CoralIntake.shootCoral());

    // Coral, RB & RT
    Constants.operatorController.rightBumper().whileTrue(m_CoralIntake.runCoral(.3))
      .whileFalse(m_CoralIntake.runCoral(0));
     // Constants.operatorController.start().onTrue(new autoShoot(m_shooter));
    Constants.operatorController.rightTrigger(.5).whileTrue(m_CoralIntake.runCoral(-.3))
     .whileFalse(m_CoralIntake.runCoral(0));

    // Algae Intake LB & LT
    Constants.operatorController.leftBumper().whileTrue(m_algaeIntake.runAlgae(.3))
      .whileFalse(m_algaeIntake.runAlgae(0));
     // Constants.operatorController.start().onTrue(new autoShoot(m_shooter));
    Constants.operatorController.leftTrigger(.5).whileTrue(m_algaeIntake.runAlgae(-.3))
     .whileFalse(m_algaeIntake.runAlgae(0));


    // Puller
    Constants.driverController.rightTrigger().whileTrue(m_puller.runPuller(-.3))
      .whileFalse(m_puller.runPuller(0));

    Constants.driverController.leftTrigger().whileTrue(m_puller.runPuller(.3))
      .whileFalse(m_puller.runPuller(0));

    //++++++++++++++++++++++ End 2025 Key Bindings +++++++++++++++++++++
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    

     
    
    
    
  
    // Default stuff remove eventually
    new JoystickButton(driverXbox.getHID(), 1).onTrue(new InstantCommand(drivebase::zeroGyro));
    
    //new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    //new JoystickButton(driverXbox, 2).whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
    //new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  
  public void configurePathPlanner() {
    // Conv
    //NamedCommands.registerCommand("runConv", m_conv.autoRunConv());
    //NamedCommands.registerCommand("stopConv", m_conv.stopConv());
    //NamedCommands.registerCommand("auton shoot", new autoShoot(m_shooter));
   

    //new ones
    //Need to see if these work, if so evaluate if we need to break them down further,
    // EX. Stopshooter, setspeed, etc...
    // Intake
      

    //NamedCommands.registerCommand("runIntake", new runAlgaeIntake(m_intake));

    // Shooter 
    //NamedCommands.registerCommand("runShooter", new autoCoral(m_shooter));

    


   // NamedCommands.registerCommand("stopIntake", m_intake.stopIntake());

   // NamedCommands.registerCommand("runIntake", m_intake.autoRunIntake());
   // NamedCommands.registerCommand("stopIntake", m_intake.stopIntake());
    // Shooter
   // NamedCommands.registerCommand("runShooter", m_shooter.autoShooterRunv1());
   // NamedCommands.registerCommand("stopShooter", m_shooter.stopShooter());
   
   //2025 Commands--------------------------------------------
   NamedCommands.registerCommand("MoveElevatorToHome", new positionElevator(m_elevator, m_elevator.BOTTOM_POSITION));
   NamedCommands.registerCommand("MoveElevatorToCoralPickup", new positionElevator(m_elevator, m_elevator.CORAL_PICKUP));
   NamedCommands.registerCommand("MoveElevatorToCoralLowBranch", new positionElevator(m_elevator, m_elevator.CORAL_LOW_BRANCH));
   NamedCommands.registerCommand("MoveElevatorToCoralMidBranch", new positionElevator(m_elevator, m_elevator.CORAL_MID_BRANCH));
   NamedCommands.registerCommand("MoveElevatorToCoralHighBranch", new positionElevator(m_elevator, m_elevator.CORAL_HIGH_BRANCH));
   NamedCommands.registerCommand("MoveElevatorToCoralBaseTrough", new positionElevator(m_elevator, m_elevator.CORAL_BASE_TROUGH));
   NamedCommands.registerCommand("MoveElevatorToAlgaePickup", new positionElevator(m_elevator, m_elevator.ALGAE_PICKUP));
   NamedCommands.registerCommand("MoveElevatorToAlgaeProcessor", new positionElevator(m_elevator, m_elevator.ALGAE_PROCESSOR));
   NamedCommands.registerCommand("MoveElevatorToAlgaeBargeNet", new positionElevator(m_elevator, m_elevator.ALGAE_BARGE_NET));
   //2025 Commands--------------------------------------------


    drivebase.setupPathPlanner();




  }

   private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    

    //DriverStation.silenceJoystickConnectionWarning(true);

    //NamedCommands.registerCommand("test", Commands.print("I EXIST"));


    // Configure the trigger bindings

   
    autoChooser = AutoBuilder.buildAutoChooser("Simple Auto");
    SmartDashboard.putData("Auto Mode", autoChooser);
    configurePathPlanner(); // Ensure AutoBuilder is set up first
    configureBindings();

    

   
    //PortForwarder.add(5800, "photonvision.local", 5800);

        // Get the voltage going into the PDP, in Volts.
    // The PDP returns the voltage in increments of 0.05 Volts.
    //double voltage = PDH.getVoltage();
    //SmartDashboard.putNumber("Voltage", voltage);

   
  }

  public void setDriveMode()
  {
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    drivebase.setDefaultCommand( !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngle );
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public Command getAutonomousCommand()
  {
   // An example command will be run in autonomous
   
    
  // timer.reset();
  // timer.start();

    //if (timer.get() > 0 && timer.get() < 1 ) {
  //     m_shooter.runShooter(-1);
   //  }

//     if (timer.get() > 1 && timer.get() < 3 ) {
//       m_intake.autoRunIntake();
      
//     }
//     if (timer.get() > 3 && timer.get() < 4 ) {
//       m_shooter.stopShooter();
//       m_intake.stopIntake();
//     }

//return m_autoShootnote;

 return drivebase.getAutonomousCommand("New Path");

 }

}
