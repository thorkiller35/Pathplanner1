package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.intakesub;
import frc.robot.subsystems.armsub;
import java.util.function.Supplier;

public class armcmd extends Command {
    private final armsub ArmSub;
    private final PIDController pidController;
    

    public armcmd(armsub ArmSub, double setpoint) {
            
        this.ArmSub = ArmSub;
        this.pidController = new PIDController(0.08, 0, 0);
        pidController.setSetpoint(setpoint);

        addRequirements(ArmSub);


    }
    @Override
    public void initialize() {
        System.out.println("IntakeSetCmd started!");
        pidController.reset();
    }

    @Override
    public void execute() {
        
        double speedF = pidController.calculate(ArmSub.getEncoderF());
        double speedB = pidController.calculate(ArmSub.getEncoderB());
        ArmSub.setMotorF(speedF);
        ArmSub.setMotorB(speedB);
        
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("IntakeSetCmd ended!");
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }


    
    
}
