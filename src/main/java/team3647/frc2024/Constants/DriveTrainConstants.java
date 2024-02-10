package team3647.frc2024.Constants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class DriveTrainConstants {
    // Declares the 2 motors. the deviceID is the can ID of the device, and can be cheked in Rev
    // Hardware Cilent
    // The motorType for NEOs is kBrushless
    public static final CANSparkMax kLeftMotor = new CANSparkMax(5, MotorType.kBrushless);
    public static final CANSparkMax kRightMotor = new CANSparkMax(4, MotorType.kBrushless);

    public static final ADIS16470_IMU gyro = new ADIS16470_IMU();

    public static final double kNominalVoltage = 12;

    public static final double kTrackWidth = 0;

    public static final double kMotorRotToWheelRot = 60 / 14;

    public static final double kWheelRadiusM = Units.inchesToMeters(4);

    public static final double motorRotationsToDistM =
            kMotorRotToWheelRot * kWheelRadiusM * 2 * Math.PI;

    public static final DifferentialDriveKinematics kinematics =
        new DifferentialDriveKinematics(kTrackWidth);

    static {
        kRightMotor.setInverted(true);
        kLeftMotor.setIdleMode(IdleMode.kBrake);
        kRightMotor.setIdleMode(IdleMode.kBrake);
    }
}
