package team3647.frc2024.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class DriveTrainConstants {
    // Declares the 2 motors. the deviceID is the can ID of the device, and can be cheked in Phoenix
    // tuner.
    public static final TalonFX kLeftMotor = new TalonFX(20);
    public static final TalonFX kRightMotor = new TalonFX(32);

    public static final double kNominalVoltage = 12;

    // creates configuraTOR object that you apply configs to
    private static final TalonFXConfigurator kleftConfigurator = kLeftMotor.getConfigurator();
    private static final TalonFXConfigurator krightConfigurator = kRightMotor.getConfigurator();

    // Creates configuraTION objects that hold the configs for each motor
    private static final TalonFXConfiguration configRight = new TalonFXConfiguration();
    private static final TalonFXConfiguration configLeft = new TalonFXConfiguration();

    public static final ADIS16470_IMU gyro = new ADIS16470_IMU();

    public static final double driveGearRatio = 14 / 60;

    public static final double kWheelRadius = 4; // ?

    // The properties of these configuraTION objects are set and applied to the ConfiguraTORs
    // static means it always runs when built
    static {
        configRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kleftConfigurator.apply(configLeft);
        krightConfigurator.apply(configRight);
    }
}
