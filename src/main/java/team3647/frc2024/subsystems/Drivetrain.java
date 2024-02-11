package team3647.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import team3647.frc2024.Constants.DriveTrainConstants;
import team3647.lib.PeriodicSubsystem;

/**
 * defines fucntionality for drivetrain, what the drivetrain *can* do. This class impliments
 * periodicSubsystem, which is an abstract class that allows us to consolidate our outputs and
 * inputs into seperate funcitons using writePeriodicOutputs and readPeriodicInputs, which are both
 * run in Periodic()(a function that runs repeatedly)
 *
 * <p>NOTE: periodicSubsystem is not included in a new project and must be copied from another
 * robot's code.
 */
public class Drivetrain implements PeriodicSubsystem {
    private final CANSparkMax left;
    private final CANSparkMax right;

    private final ADIS16470_IMU gyro;

    private final Pose2d initialPose = new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0));

    private final DifferentialDrivePoseEstimator estimator;
    // periodicIO consolidates the measured input and output values
    private final PeriodicIO periodicIO = new PeriodicIO();

    public Drivetrain(
            CANSparkMax left,
            CANSparkMax right,
            ADIS16470_IMU gyro,
            DifferentialDriveKinematics kinematics) {
        this.left = left;
        this.right = right;
        this.gyro = gyro;
        SmartDashboard.putNumber("balls", getRightDistM());
        estimator =
                new DifferentialDrivePoseEstimator(
                        kinematics,
                        Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kYaw)),
                        getLeftDistM(),
                        getRightDistM(),
                        initialPose);
    }

    public void drive(double forward, double rotation) {
        /**
         * wheelspeeds represents the speeds of eaach wheel/motor in a differential drive. it takes
         * left and right speed arcadeDriveIK creates wheelspeeds with a forward and rotaion
         * component. it calculates left and right speed by adding rotation to forward for right,
         * and subtracing rotation from forward for left.
         */
        WheelSpeeds ws = DifferentialDrive.arcadeDriveIK(forward, rotation, false);
        setOpenloop(ws.left, ws.right);
        Logger.recordOutput("drive/arcadeForward", forward);
        Logger.recordOutput("drive/arcadeRot", rotation);
    }

    public void setOpenloop(double leftOutput, double rightOutput) {
        // sets values of rightoutput
        periodicIO.leftOutput = leftOutput;
        periodicIO.rightOutput = rightOutput;

        Logger.recordOutput("leftOut", leftOutput);
        Logger.recordOutput("right output", rightOutput);
    }

    public void calibrateGyro() {
        this.gyro.calibrate();
    }

    public double getRightDistM() {
        return right.getEncoder().getPosition() * DriveTrainConstants.kMotorRotationsToDistM;
    }

    public double getLeftDistM() {
        return periodicIO.inputs.leftDistM;
    }

    public double getYaw() {
        return periodicIO.inputs.yaw;
    }

    public double getPitch() {
        return periodicIO.inputs.pitch;
    }

    public double getRoll() {
        return periodicIO.inputs.roll;
    }

    public double getRightVoltage() {
        return periodicIO.inputs.rightVoltage;
    }

    public double getLeftVoltage() {
        return periodicIO.inputs.leftVoltage;
    }

    public double getLeftCurrent() {
        return periodicIO.inputs.leftCurrent;
    }

    public double getRightCurrent() {
        return periodicIO.inputs.rightCurrent;
    }

    public double getAvgCurrent() {
        return periodicIO.inputs.avgCurrent;
    }

    @Override
    public void writePeriodicOutputs() {
        // setControl() takes a controlRequest Object and applies it's output to the motor
        this.left.set(periodicIO.leftOutput);
        this.right.set(periodicIO.rightOutput);
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.inputs.leftVoltage = this.left.getBusVoltage();
        periodicIO.inputs.rightVoltage = this.right.getBusVoltage();

        periodicIO.inputs.leftVel = this.left.getEncoder().getVelocity();
        periodicIO.inputs.rightVel = this.left.getEncoder().getVelocity();

        periodicIO.inputs.yaw = this.gyro.getAngle(IMUAxis.kYaw);
        periodicIO.inputs.pitch = this.gyro.getAngle(IMUAxis.kPitch);
        periodicIO.inputs.roll = this.gyro.getAngle(IMUAxis.kRoll);
        periodicIO.inputs.leftCurrent = left.getOutputCurrent();
        periodicIO.inputs.rightCurrent = right.getOutputCurrent();
        periodicIO.inputs.avgCurrent = (right.getOutputCurrent() + left.getOutputCurrent()) / 2;

        periodicIO.inputs.rightDistM =
                right.getEncoder().getPosition() * DriveTrainConstants.kMotorRotationsToDistM;
        periodicIO.inputs.leftDistM =
                left.getEncoder().getPosition() * DriveTrainConstants.kMotorRotationsToDistM;

        periodicIO.inputs.leftNativePos = left.getEncoder().getPosition();
        periodicIO.inputs.rightNativePos = right.getEncoder().getPosition();

        this.estimator.update(
                Rotation2d.fromDegrees(getYaw()),
                new DifferentialDriveWheelPositions(getLeftDistM(), getRightDistM()));
        periodicIO.inputs.odoPose = this.estimator.getEstimatedPosition();

        Logger.processInputs("Drive/inputs", periodicIO.inputs);
    }

    public static class PeriodicIO {
        public double feedforward = 1;
        public double nominalVoltage = 12;
        public double leftOutput;
        public double rightOutput;

        public Inputs inputs = new Inputs();

        public class Inputs implements LoggableInputs {
            @Override
            public void toLog(LogTable table) {
                table.put("leftVoltage", leftVoltage);
                table.put("rightVoltage", rightVoltage);
                table.put("leftOpenLoop", leftOpenLoop);
                table.put("rightopenloop", rightOpenLoop);
                table.put("leftvel", leftVel);
                table.put("yaw", yaw);
                table.put("pitch", pitch);
                table.put("roll", roll);
                table.put("current pose", odoPose);
                table.put("leftCurrent", leftCurrent);
                table.put("rightcurret", rightCurrent);
                table.put("right distance m", rightDistM);
                table.put("chill", leftDistM);
                table.put("rigthNativePos", rightNativePos);
                table.put("left nat pos", leftNativePos);
            }

            @Override
            public void fromLog(LogTable table) {
                leftVel = table.get("leftvel", -1);
                rightVel = table.get("right velocity", -1);
            }

            public double leftVoltage = 0;
            public double rightVoltage = 0;

            public double leftVel = 0;
            public double rightVel = 0;

            public double leftOpenLoop = 0;
            public double rightOpenLoop = 0;

            public double timestamp = 0;

            public double rightCurrent = 0;
            public double leftCurrent = 0;
            public double avgCurrent = 0;

            public double yaw = 0;
            public double pitch = 0;
            public double roll = 0;

            public double rightDistM = 0;
            public double leftDistM = 0;

            public double leftNativePos = 0;
            public double rightNativePos = 0;

            public Pose2d odoPose = new Pose2d();
        }
    }

    @Override
    public void periodic() {
        readPeriodicInputs();
        writePeriodicOutputs();
    }

    @Override
    public String getName() {
        return "Drivetrain";
    }
}
