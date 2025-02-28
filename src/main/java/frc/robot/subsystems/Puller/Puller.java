package frc.robot.subsystems.Puller;

// 1 motor

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;

public class Puller extends SubsystemBase {
    private SparkMax pullerMotor;
    private SparkMaxConfig pullerMotorConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;


    public Puller() {
        /*
        * Initialize the SPARK MAX and get its encoder and closed loop controller
        * objects for later use.
        */
        pullerMotorConfig = new SparkMaxConfig();

        pullerMotorConfig.idleMode(IdleMode.kCoast);
        pullerMotor = new SparkMax(Constants.PullerID, MotorType.kBrushless);
        closedLoopController = pullerMotor.getClosedLoopController();
        encoder = pullerMotor.getEncoder();
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
    pullerMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    pullerMotorConfig.closedLoop
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
    pullerMotor.configure(pullerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    }

   
    


    public Command runPuller(double speed){
        return startEnd(
            () -> pullerMotor.set(speed),
            () -> pullerMotor.stopMotor()
        );
    }
    public Command runIntake(){
        return runPuller(0.3);
    }

    // Auto
    public Command autoRunPuller(){
        return runPuller(-0.3);
    }
    public Command autoPullerUp(){
        return this.runOnce(
            () -> pullerMotor.set(.3)
        );
    }

    public Command autoCoralDown(){
        return this.runOnce(
            () -> pullerMotor.set(-.2)
        );
    }


    public Command stopCoral(){
        return this.runOnce(
            () -> pullerMotor.stopMotor()
        );
    } 
}