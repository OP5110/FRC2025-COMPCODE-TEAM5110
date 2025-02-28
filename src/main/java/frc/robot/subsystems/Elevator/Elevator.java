package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private SparkMax elevatorMotor;
    private SparkMax elevatorMotor2;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;

    // **Preset Positions for Coral and Algae **
    public static final double BOTTOM_POSITION = 0.0; // **Home Position**
    
    // **Coral Positions**
    public static final double CORAL_PICKUP = 0;        // **Picking up Coral from the ground**
    public static final double CORAL_LOW_BRANCH = 0;   // **Scoring on Low Reef Branch**
    public static final double CORAL_MID_BRANCH = 29.33;   // **Scoring on Mid Reef Branch**
    public static final double CORAL_HIGH_BRANCH = 76.79;  // **Scoring on High Reef Branch**
    public static final double CORAL_BASE_TROUGH = 87.91;  // **Scoring in Base Trough**

    // **Algae Positions**
    public static final double ALGAE_PICKUP = 5.0;       // **Picking up Algae**
    public static final double ALGAE_PROCESSOR = 20.0;   // **Delivering to Algae Processor**
    public static final double ALGAE_BARGE_NET = 30.0;   // **Scoring in Barge Net**
    public static final double ELEVATOR_TOP = 86.91; 


    public Elevator() {
        // Initialize Motors
        elevatorMotor = new SparkMax(Constants.ElevatorIDLeft, MotorType.kBrushless);
        elevatorMotor2 = new SparkMax(Constants.ElevatorIDRight, MotorType.kBrushless);
    
        // Leader Motor Configuration
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(IdleMode.kBrake);
        leaderConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.05).i(0).d(0).outputRange(-0.35, 0.35)
            .velocityFF(0.00015);
    
        // Follower Motor Configuration (Independent but Mirroring)
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kBrake);
        followerConfig.inverted(true); // Flip direction if needed
        followerConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        followerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.05).i(0).d(0).outputRange(-0.35, 0.35)
            .velocityFF(0.00015); 
    
        // Apply Configurations
        elevatorMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        elevatorMotor2.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
        // Get Encoders and PID Controllers for Each Motor
        encoder = elevatorMotor.getEncoder();
        RelativeEncoder encoder2 = elevatorMotor2.getEncoder();
    
        closedLoopController = elevatorMotor.getClosedLoopController();
        SparkClosedLoopController closedLoopController2 = elevatorMotor2.getClosedLoopController();
    
        // Reset Encoder Positions
        encoder.setPosition(0);
        encoder2.setPosition(0);
    
        // SmartDashboard Display
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
        SmartDashboard.setDefaultNumber("Target Position", 0);
        SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    }

    

     /** MOVE TO PRESET POSITION **/

     public Command moveToPosition(double position) {
        return new InstantCommand(() -> {
            closedLoopController.setReference(position, ControlType.kPosition);
            elevatorMotor2.getClosedLoopController().setReference(position, ControlType.kPosition);
        }, this);
    }
    

  
    /** PRESET COMMANDS **/
    public Command moveToHome() {
        return moveToPosition(BOTTOM_POSITION);
        
    }

    // **Coral Commands**
    public Command moveToCoralPickup() {
        return moveToPosition(CORAL_PICKUP);
    }
    public Command moveToCoralLowBranch() {
        return moveToPosition(CORAL_LOW_BRANCH);
    }
    public Command moveToCoralMidBranch() {
        return moveToPosition(CORAL_MID_BRANCH);
    }
    public Command moveToCoralHighBranch() {
        return moveToPosition(CORAL_HIGH_BRANCH);
    }
    public Command moveToCoralBaseTrough() {
        return moveToPosition(CORAL_BASE_TROUGH);
    }

    // **Algae Commands**
    public Command moveToAlgaePickup() {
        return moveToPosition(ALGAE_PICKUP);
    }
    public Command moveToAlgaeProcessor() {
        return moveToPosition(ALGAE_PROCESSOR);
    }
    public Command moveToAlgaeBargeNet() {
        return moveToPosition(ALGAE_BARGE_NET);
    }


    //-------------------------------------------------------------
    //-------------------------------------------------------------
    /** MANUAL CONTROL MOVEMENT **/
    //  public Command runElevator(double speed) {
    //      return new InstantCommand(() -> {
    //          elevatorMotor.set(speed);
    //          elevatorMotor2.set(speed);
    //      });
    //  }

    public Command runElevator(double speed) {
       return new InstantCommand(() -> {
           double currentPosition = encoder.getPosition(); // Replace with actual method to get position
           
           // Prevent movement beyond ALGAE_BARGE_NET when moving up
           if (speed > 0 && currentPosition >= ELEVATOR_TOP) {
               elevatorMotor.set(0.02); // Hold position
               elevatorMotor2.set(0.02);
           } 
           // Otherwise, allow movement
           else {
               elevatorMotor.set(speed);
               elevatorMotor2.set(speed);
           }
       });
   }
    



    /** STOP ELEVATOR **/
    public Command stopElevator() {
        return new InstantCommand(() -> {
            elevatorMotor.set(0);
            elevatorMotor2.set(0);
        });
    }

    public double getCurrentPosition() {
        return encoder.getPosition();
    }

    @Override
    public void periodic() {
        // **Update SmartDashboard with encoder position**
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
        SmartDashboard.putNumber("Motor1 Output", elevatorMotor.get());
        SmartDashboard.putNumber("Motor2 Output", elevatorMotor2.get());

        
        //elevatorMotor2.set(elevatorMotor.get());
    }
}