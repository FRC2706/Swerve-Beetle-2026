package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroGyroCommand extends Command {
    private final SwerveSubsystem m_SwerveDrive;

    public ZeroGyroCommand(SwerveSubsystem swerveDrive) {
        m_SwerveDrive = swerveDrive;
        
        addRequirements(m_SwerveDrive);
    }

    @Override
    public void initialize() {
        m_SwerveDrive.zeroGyro();
        // Initialization logic for the command
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        // Return true when the command should end
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        
    } 
}
