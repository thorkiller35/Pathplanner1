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


public class armsub extends SubsystemBase{
    private CANSparkMax pivfront = new CANSparkMax(13, MotorType.kBrushless);
    private CANSparkMax pivback = new CANSparkMax(10,MotorType.kBrushless);


    private RelativeEncoder pivfrontenc = pivfront.getEncoder();
    private RelativeEncoder pivbackenc = pivback.getEncoder();

    public armsub(){


    }
    @Override
    public void periodic() {
    }
    public void setMotorF(double speed){
        pivfront.set(speed);
       

        
    }
    public void setMotorB(double speed){
       
        pivback.set(speed);

        
    }
    public double getEncoderF(){
        return pivfrontenc.getPosition();
    }
    public double getEncoderB(){
        return pivbackenc.getPosition();

    }


}

