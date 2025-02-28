package frc.robot.subsystems.AlgaeIntake;
// 2 Motors, Spinning in opposite directions to intake the balls, other motor to nudge balls into the intake/shooter
// Will have to be able to read the encoder for angle of the intake

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

public class AlgaeIntake extends SubsystemBase {
    private SparkMax algaeMotor;
    private SparkMax algaeMotor2;
    private SparkMaxConfig algaeMotorConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;


    public AlgaeIntake() {
        /*
        * Initialize the SPARK MAX and get its encoder and closed loop controller
        * objects for later use.
        */
        algaeMotorConfig = new SparkMaxConfig();

        algaeMotorConfig.idleMode(IdleMode.kCoast);
        algaeMotor = new SparkMax(Constants.AlgaeIntakeIDLeft, MotorType.kBrushless);
        algaeMotor2 = new SparkMax(Constants.AlgaeIntakeIDRight, MotorType.kBrushless);
        closedLoopController = algaeMotor.getClosedLoopController();
        closedLoopController = algaeMotor2.getClosedLoopController();
        encoder = algaeMotor.getEncoder();
        encoder = algaeMotor2.getEncoder();
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
    algaeMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    algaeMotorConfig.closedLoop
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
    algaeMotor.configure(algaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    algaeMotor2.configure(algaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    }

    // Method to run the shooter motors with a given speed using a Command
    public Command runAlgae(double speed) {
        return new InstantCommand(() -> {
            // Run both motors at the specified speed
            algaeMotor.set(-speed);
            algaeMotor2.set(speed);
        });
    }

    // Autonomous methods to run the shooter
    public void autonrunAlgae(double speed) {
        System.out.println("In Shooter subsystem");
        algaeMotor.set(speed);
        algaeMotor2.set(speed);
    }

    // Autonomous method to stop the shooter motors
    public void autonstopAlgae() {
        algaeMotor.set(0);
        algaeMotor2.set(0);
    }

    // Command to stop the shooter motors
    public Command stopAlage() {
        return new InstantCommand(() -> {
            algaeMotor.set(0);
            algaeMotor2.set(0);
        });
    }

    // Auton version 1 for the shooter with motor profiles applied
    public Command autoShooterRunv1A() {
        return new InstantCommand(() -> {
            // Apply specific profile (this could be a more advanced profile)
            algaeMotor.set(-0.85); // Direct set speed (profile applied through ramp rates)
            algaeMotor2.set(-0.85); // Direct set speed (profile applied through ramp rates)
        });
    }

    // Auton version 2 for the shooter with a different profile
    public Command autoShooterRunv2() {
        return new InstantCommand(() -> {
            // Apply specific profile (same as version 1, can be adjusted)
            algaeMotor.set(-0.85); // Direct set speed (profile applied through ramp rates)
            algaeMotor2.set(-0.85); // Direct set speed (profile applied through ramp rates)
        });
    }
    
}
