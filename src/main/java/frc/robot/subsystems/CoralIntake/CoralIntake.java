package frc.robot.subsystems.CoralIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
    private SparkMax coralMotor;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;
    
    // Constants for shooting and reversing
    public static final double SHOOT = 1.0;
    public static final double REVERSE = -0.3;
    
    public CoralIntake() {
        coralMotor = new SparkMax(Constants.CoralIntakeID, MotorType.kBrushless);
        
        // Motor Configuration
        SparkMaxConfig coralMotorConfig = new SparkMaxConfig();
        coralMotorConfig.idleMode(IdleMode.kCoast);
        coralMotorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        coralMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.05).i(0).d(0).outputRange(-0.35, 0.35)
            .velocityFF(0.00015);
        
        coralMotor.configure(coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        encoder = coralMotor.getEncoder();
        closedLoopController = coralMotor.getClosedLoopController();
        
        // SmartDashboard Initialization
        SmartDashboard.putNumber("Coral Motor Output", coralMotor.get());
        SmartDashboard.putNumber("Coral Position", encoder.getPosition());
        SmartDashboard.setDefaultNumber("Target Position", 0);
    }
    
    /** Runs the coral intake at a given speed */
    public Command runCoral(double speed) {
        return new InstantCommand(() -> coralMotor.set(speed), this);
    }
    
    /** Moves to a specific position using PID control */
    public Command moveToPosition(double position) {
        return new InstantCommand(() -> {
            closedLoopController.setReference(position, com.revrobotics.spark.SparkBase.ControlType.kPosition);
        }, this);
    }
    
    /** Shoots the coral */
    public Command shootCoral() {
        return moveToPosition(SHOOT);
    }
    
    /** Reverses the coral intake */
    public Command reverseCoral() {
        return moveToPosition(REVERSE);
    }
    
    /** Stops the coral intake */
    public Command stopCoral() {
        return new InstantCommand(() -> coralMotor.stopMotor(), this);
    }
    
    /** Autonomous coral intake command */
    public Command autonRunCoral() {
        return moveToPosition(REVERSE);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Coral Motor Output", coralMotor.get());
        SmartDashboard.putNumber("Coral Position", encoder.getPosition());
    }
}

