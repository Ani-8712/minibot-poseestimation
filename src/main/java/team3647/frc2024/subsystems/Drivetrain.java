package team3647.frc2024.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import team3647.frc2024.Constants.DriveTrainConstants;
import team3647.frc2024.Constants.FieldConstants;
import team3647.frc2024.util.VisionData;
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
    private final SparkPIDController leftController;
    private final SparkPIDController rightController;

    private final ADIS16470_IMU gyro;

    private final Pose2d initialPose = new Pose2d(2.71, 6.99, Rotation2d.fromDegrees(180));

    private final DifferentialDrivePoseEstimator estimator;
    // periodicIO consolidates the measured input and output values
    private final PeriodicIO periodicIO = new PeriodicIO();

    private final AprilTagFieldLayout layout =
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public Drivetrain(
            CANSparkMax left,
            CANSparkMax right,
            ADIS16470_IMU gyro,
            DifferentialDriveKinematics kinematics) {
        this.left = left;
        this.right = right;
        this.gyro = gyro;
        this.leftController = left.getPIDController();
        this.rightController = right.getPIDController();

        estimator =
                new DifferentialDrivePoseEstimator(
                        kinematics,
                        Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kYaw)),
                        getLeftDistM(),
                        getRightDistM(),
                        initialPose);
    }

    public void drive(double forward, double rotation, boolean isOpenLoop) {
        /**
         * wheelspeeds represents the speeds of eaach wheel/motor in a differential drive. it takes
         * left and right speed arcadeDriveIK creates wheelspeeds with a forward and rotaion
         * component. it calculates left and right speed by adding rotation to forward for right,
         * and subtracing rotation from forward for left.
         */
        WheelSpeeds ws = DifferentialDrive.arcadeDriveIK(forward, -rotation, false);

        if (isOpenLoop) {
            setOpenloop(ws.left, ws.right);
        } else {
            setVelocity(ws.left, ws.right);
        }

        Logger.recordOutput("drive/arcadeForward", forward);
        Logger.recordOutput("drive/arcadeRot", rotation);
    }

    public void setVelocity(double leftVel, double rightVel) {
        periodicIO.leftOutput = leftVel;
        periodicIO.rightOutput = rightVel;
        periodicIO.controlMode = ControlType.kDutyCycle;

        Logger.recordOutput("Drive/ leftvel", leftVel);
        Logger.recordOutput("drive/rightvel", rightVel);
    }

    public void setOpenloop(double leftOutput, double rightOutput) {
        // sets values of rightoutput
        periodicIO.leftOutput = leftOutput;
        periodicIO.rightOutput = rightOutput;
        periodicIO.controlMode = ControlType.kDutyCycle;

        Logger.recordOutput("leftOpenLoop", leftOutput);
        Logger.recordOutput("right open loop", rightOutput);
    }

    public void addVisionData(VisionData data) {
        addVisionData(data.pose, data.timestamp, data.stdDevs);
    }

    public void addVisionData(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        periodicIO.visionPose = pose;
        this.estimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    public double getAngleToSpeaker() {
        var x = periodicIO.odoPose.minus(FieldConstants.kBlueSpeakerPose).getX();
        var y = periodicIO.odoPose.minus(FieldConstants.kBlueSpeakerPose).getY();
        var angle = Math.atan2(y, x);
        return Units.radiansToDegrees(angle) + 180;
    }

    public double getOdoRotReversed(){
        return (periodicIO.odoPose.getRotation().getDegrees()+180)%360;
    }

    public void calibrateGyro() {
        this.gyro.calibrate();
    }

    public void zeroGyro(){
        this.gyro.setGyroAngle(IMUAxis.kYaw, this.initialPose.getRotation().getDegrees());
    }

    public double getRightDistM() {
        return periodicIO.rightDistM;
    }

    public double getLeftDistM() {
        return periodicIO.leftDistM;
    }

    public double getYaw() {
        return periodicIO.yaw;
    }

    public double getPitch() {
        return periodicIO.pitch;
    }

    public double getRoll() {
        return periodicIO.roll;
    }

    public double getRightVoltage() {
        return periodicIO.rightVoltage;
    }

    public double getLeftVoltage() {
        return periodicIO.leftVoltage;
    }

    public double getLeftCurrent() {
        return periodicIO.leftCurrent;
    }

    public double getRightCurrent() {
        return periodicIO.rightCurrent;
    }

    public double getAvgCurrent() {
        return periodicIO.avgCurrent;
    }

    @Override
    public void writePeriodicOutputs() {
        // setControl() takes a controlRequest Object and applies it's output to the motor
        this.leftController.setReference(periodicIO.leftOutput, periodicIO.controlMode);
        this.rightController.setReference(periodicIO.rightOutput, periodicIO.controlMode);
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.leftVoltage = this.left.getBusVoltage();
        periodicIO.rightVoltage = this.right.getBusVoltage();

        periodicIO.leftVel = this.left.getEncoder().getVelocity();
        periodicIO.rightVel = this.left.getEncoder().getVelocity();

        periodicIO.yaw = this.gyro.getAngle(IMUAxis.kYaw);
        periodicIO.pitch = this.gyro.getAngle(IMUAxis.kPitch);
        periodicIO.roll = this.gyro.getAngle(IMUAxis.kRoll);
        periodicIO.leftCurrent = left.getOutputCurrent();
        periodicIO.rightCurrent = right.getOutputCurrent();
        periodicIO.avgCurrent = (right.getOutputCurrent() + left.getOutputCurrent()) / 2;

        periodicIO.rightDistM =
                right.getEncoder().getPosition() * DriveTrainConstants.kMotorRotationsToDistM;
        periodicIO.leftDistM =
                left.getEncoder().getPosition() * DriveTrainConstants.kMotorRotationsToDistM;

        periodicIO.leftNativePos = left.getEncoder().getPosition();
        periodicIO.rightNativePos = right.getEncoder().getPosition();

        periodicIO.timestamp = Timer.getFPGATimestamp();

        this.estimator.update(
                Rotation2d.fromDegrees(getYaw()),
                new DifferentialDriveWheelPositions(getLeftDistM(), getRightDistM()));
        periodicIO.odoPose = this.estimator.getEstimatedPosition();

        // periodicIO.stdDevsScalar =
        //         GeomUtil.distance(
        //                 this.layout
        //                         .getTagPose((int) (LimelightHelpers.getFiducialID("")))
        //                         .get()
        //                         .toPose2d(),
        //                 periodicIO.visionPose);

        // addVisionData(LimelightHelpers.getBotPose2d(""),
        //             periodicIO.timestamp,
        //             VecBuilder.fill(0.005,0.005, 0.005)
        //             .times(periodicIO.stdDevsScalar));

        Logger.processInputs("Drive/inputs", periodicIO);
    }

    public static class PeriodicIO implements LoggableInputs {
        public double feedforward = 1;
        public double nominalVoltage = 12;
        public double leftOutput;
        public double rightOutput;
        public ControlType controlMode = ControlType.kDutyCycle;

        public double stdDevsScalar = 1;

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
            table.put("odo pose", odoPose);
            table.put("vision pose", visionPose);
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

        public Pose2d visionPose = new Pose2d();
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
