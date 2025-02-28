
package frc.robot.commands.elevatorcommands;
//package frc.robot.commands.IntakeCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;


public class positionElevator extends Command {
    private final Elevator m_autoelevator;
    private final double targetPosition;
    private final double tolerance = 1.0; // Allowable error

    public positionElevator(Elevator elevator, double targetPosition) {
        this.m_autoelevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(m_autoelevator);
    }

    @Override
    public void initialize() {
        m_autoelevator.moveToPosition(targetPosition).schedule();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_autoelevator.getCurrentPosition() - targetPosition) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        m_autoelevator.stopElevator().schedule();
    }
}