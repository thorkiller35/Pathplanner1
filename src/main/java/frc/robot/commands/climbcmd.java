package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakesub;
import frc.robot.subsystems.climbsub;

public class climbcmd extends Command {

    private final climbsub climbsubsy;
    private final double speed;



    public climbcmd(climbsub climbsubsy, Double speed) {

        this.climbsubsy = climbsubsy;
        this.speed = speed;
     
        addRequirements(climbsubsy);
    }

    @Override
    public void initialize() {
        System.out.println("IntakeSetCmd started!");
    }

    @Override
    public void execute() {
        climbsubsy.setMotors(speed);
    }

    @Override
    public void end(boolean interrupted) {
        climbsubsy.setMotors(0);

        System.out.println("IntakeSetCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}