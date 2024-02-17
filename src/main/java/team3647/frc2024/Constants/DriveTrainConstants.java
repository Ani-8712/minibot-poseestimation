package team3647.frc2024.Constants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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

    public static final double kMotorRotToWheelRot = 12 / 60.0;

    public static final double kDriveP = 0.5;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kWheelDiameterM = Units.inchesToMeters(4);

    public static final double kMotorRotationsToDistM =
            kMotorRotToWheelRot * kWheelDiameterM * Math.PI;

    public static final DifferentialDriveKinematics kinematics =
            new DifferentialDriveKinematics(kTrackWidth);

    public static final ProfiledPIDController kRotController =
            new ProfiledPIDController(0.5, 0, 0, new Constraints(0.7, 0.3));

    static {
        kLeftMotor.setInverted(true);
        kRightMotor.setInverted(false);
        kLeftMotor.setIdleMode(IdleMode.kBrake);
        kRightMotor.setIdleMode(IdleMode.kBrake);

        kLeftMotor.getPIDController().setP(kDriveP);
        kLeftMotor.getPIDController().setI(kDriveI);
        kLeftMotor.getPIDController().setD(kDriveD);

        kRightMotor.getPIDController().setP(kDriveP);
        kRightMotor.getPIDController().setI(kDriveI);
        kRightMotor.getPIDController().setD(kDriveD);
        kRotController.enableContinuousInput(-180, 180);
    }
}
