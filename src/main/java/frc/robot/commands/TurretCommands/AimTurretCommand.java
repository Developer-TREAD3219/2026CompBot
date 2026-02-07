package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

public class AimTurretCommand extends Command {
    private final TurretSubsystem m_turret;

    public AimTurretCommand(TurretSubsystem subsystem) {
        m_turret = subsystem;
        addRequirements(m_turret);
    }

    @Override
    public void execute() {
        // Get the calculated motor speed from the subsystem logic
        double speed = m_turret.calculateTurretCommand();
        // Apply the speed to the motor
        m_turret.turnTurret(speed);
    }

    @Override
    public boolean isFinished() {
        // Command finishes when the target is centered within a small tolerance
        return m_turret.hasTarget() && Math.abs(m_turret.getTx()) < TurretConstants.kTargetToleranceDegrees;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.turnTurret(0); // Stop the turret when command ends
    }
}
