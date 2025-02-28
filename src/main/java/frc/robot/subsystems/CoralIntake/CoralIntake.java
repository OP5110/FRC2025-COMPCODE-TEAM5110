package frc.robot.subsystems.CoralIntake;
// 1 motor to intake balls and shoot them

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
    private SparkMax coralMotor;
    private SparkMaxConfig coralMotorConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;


    public CoralIntake() {
        /*
        * Initialize the SPARK MAX and get its encoder and closed loop controller
        * objects for later use.
        */
        coralMotorConfig = new SparkMaxConfig();

        coralMotorConfig.idleMode(IdleMode.kCoast);
        coralMotor = new SparkMax(Constants.CoralIntakeID, MotorType.kBrushless);
        closedLoopController = coralMotor.getClosedLoopController();
        encoder = coralMotor.getEncoder();
    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    coralMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    coralMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 10)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    coralMotor.configure(coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    }


    // Coral Shooter Commands
   //notdone
    public Command runCoral(double speed){
        return startEnd(
            () -> coralMotor.set(speed),
            () -> coralMotor.stopMotor()
        );
    }
    public Command runCoralIntake(){
        return runCoral(0.3);
    }

    // Auto
    public Command autonRunCoral(){
        return runCoral(-0.3);
    }
    public Command autonCoralUp(){
        return this.runOnce(
            () -> coralMotor.set(.3)
        );
    }

    public Command autonCoralDown(){
        return this.runOnce(
            () -> coralMotor.set(-.2)
        );
    }


    public Command stopCoral(){
        return this.runOnce(
            () -> coralMotor.stopMotor()
        );
    } 
}

