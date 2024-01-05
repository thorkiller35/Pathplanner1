package frc.robot.subsystems;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
 
 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
 
 
import edu.wpi.first.wpilibj.Joystick;

import java.util.function.Supplier;
import frc.robot.commands.climbcmd;



public class climbsub extends SubsystemBase{
    private CANSparkMax climb = new CANSparkMax(5, MotorType.kBrushless);



    private RelativeEncoder climbencoder = climb.getEncoder();
  

    public climbsub(){


    }
    @Override
    public void periodic() {
    }
    public void setMotorF(double speed){
        
       

        
    }
    public void set(double speed){
        climb.set(speed);
       
        

        
    }

    public void setMotors(double speed){
        climb.set(speed);
    }
    public double getEncoder(){
        return climbencoder.getPosition();
    }
   

}

