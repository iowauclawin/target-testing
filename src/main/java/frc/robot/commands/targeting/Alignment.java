package frc.robot.commands.targeting;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.targeting.Vision;


public class Alignment extends Command {
    // SwerveDrive swerve;
    Vision vision;
    PIDController pid = new PIDController(0, 0 , 0); //TODO figure out PID constants por favor

    public Alignment( Vision vision) {
        // this.swerve = swerve;
        this.vision = vision;
    }
    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
