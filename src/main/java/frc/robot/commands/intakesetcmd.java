package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakesub;

public class intakesetcmd extends Command {

    private final intakesub intakeSubsystem;
    private final double speed;



    public intakesetcmd(intakesub intakeSubsystem, Double speed) {
        this.speed = speed;
        
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("IntakeSetCmd started!");
    }

    @Override
    public void execute() {
       intakeSubsystem.setMotors(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setMotors(0);

        System.out.println("IntakeSetCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}