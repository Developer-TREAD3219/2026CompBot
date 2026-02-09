package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;


// // TO DO:
//         // Get the pose of the target from the LimeLightSubsystem (or calculate the desired angle based on the target's position and the robot's pose)
//         m_targetFieldPos = LimeLightSubsystem.getTargetFieldPosition(23); // TO DO: Example tag ID for the hub, update as needed
//         // Calculate the desired motor command using the PID controller in the TurretSubsystem
//         //????
//         // Apply the motor command to the turret motor
//          m_turret.turnTurret(m_turret.getTx()); // This is a placeholder, replace with actual PID output based on target angle


public class AutoAimTurretCommand extends Command {
    private final TurretSubsystem m_turret;

    public AutoAimTurretCommand(TurretSubsystem subsystem) {
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
