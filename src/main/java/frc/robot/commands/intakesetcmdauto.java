package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakesub;

public class intakesetcmdauto extends Command {

    private final intakesub intakeSubsystem;
    private final double speed;
    private final Timer timer = new Timer();



    public intakesetcmdauto(intakesub intakeSubsystem, Double speed) {
        this.speed = speed;
        
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        System.out.println("IntakeSetCmd started!");
    }

    @Override
    public void execute() {
       intakeSubsystem.setMotors(speed);
       SmartDashboard.putNumber("time auto", timer.get());
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setMotors(0);

        System.out.println("IntakeSetCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= 2;
    }
}