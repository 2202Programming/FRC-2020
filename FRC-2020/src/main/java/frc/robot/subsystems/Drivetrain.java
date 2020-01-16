package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import static frc.robot.Constants.*;

public class Drivetrain implements Subsystem {
        private CANSparkMax frontRight = new CANSparkMax(FR_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax frontLeft = new CANSparkMax(FL_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax backRight = new CANSparkMax(BR_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax backLeft = new CANSparkMax(BL_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax middleRight = new CANSparkMax(MR_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax middleLeft = new CANSparkMax(ML_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        private DifferentialDrive drive;

        public Drivetrain() {
                middleRight.follow(frontRight);
                middleLeft.follow(frontLeft);
                backRight.follow(frontRight);
                backLeft.follow(frontLeft);
                drive = new DifferentialDrive(frontLeft, frontRight);
        }

        public void arcadeDrive(double xSpeed, double rotationSpeed) {
                drive.arcadeDrive(xSpeed, rotationSpeed);
        }
}