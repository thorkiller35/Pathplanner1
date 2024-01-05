


package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.DrivetrainConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class intakesub extends SubsystemBase {
    private CANSparkMax intakeright = new CANSparkMax(12, MotorType.kBrushless);
    private CANSparkMax intakeleft = new CANSparkMax(4,MotorType.kBrushless);
    public  intakesub() {
    

}
@Override
public void periodic(){

}
public void setMotorF(double speed){
    intakeright.set(speed);
   

    
}
public void setMotorB(double speed){
   
    intakeleft.set(speed);

    
}
// public void set_intake(boolean open){
//     if (open){
//         intakeright.set(0.5);
//         intakeleft.set(0.5);
    

//     }
//     else{
//         intakeright.set(0);
//         intakeleft.set(0);
//     }
// }
// public void set_out(boolean closed){
//         if (closed){
//             intakeright.set(-0.5);
//             intakeleft.set(-0.5);
    
//         }
//         else{
//         intakeleft.set(0);
//         intakeleft.set(0);
            
//         }
    
    
    


public void setMotors(double speed){
    intakeleft.set(speed);
    intakeright.set(speed);
}

}
