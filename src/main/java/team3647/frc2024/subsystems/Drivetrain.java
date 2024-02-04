package team3647.frc2024.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import team3647.frc2024.Constants.DriveTrainConstants;
import team3647.lib.PeriodicSubsystem;

/**
 * defines fucntionality for drivetrain, what the drivetrain *can* do. This class impliments
 * periodicSubsystem, which is an interface class that allows us to consolidate our outputs and
 * inputs into seperate funcitons using writePeriodicOutputs and readPeriodicInputs, which are both
 * run in Periodic()(a function that runs repeatedly)
 *
 * <p>NOTE: periodicSubsystem is not included in a new project and must be copied from another
 * robot's code.
 */
public class Drivetrain implements PeriodicSubsystem {
    private final TalonFX left;
    private final TalonFX right;

    private final DifferentialDriveOdometry odo;

    /**
     * DutyCycleOut is the control request for open loop (see
     * https://v6.docs.ctr-electronics.com/en/2023-v6/docs/api-reference/api-usage/control-requests.html)
     */
    private final DutyCycleOut leftDutycycle = new DutyCycleOut(0);

    private final DutyCycleOut rightDutycycle = new DutyCycleOut(0);

    private final ADIS16470_IMU gyro;

    // periodicIO consolidates the measured input and output values
    private final PeriodicIO periodicIO = new PeriodicIO();

    public Drivetrain(TalonFX left, TalonFX right, ADIS16470_IMU gyro) {
        this.left = left;
        this.right = right;
        this.gyro = gyro;

        this.odo =
                new DifferentialDriveOdometry(
                        Rotation2d.fromDegrees(this.gyro.getAngle(IMUAxis.kYaw)),
                        getLeftMeters(),
                        getRightMeters());
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
        Logger.recordOutput("Drive/forward", forward);
        Logger.recordOutput("Drive/Rotation", rotation);
    }

    public void setOpenloop(double leftOutput, double rightOutput) {
        // gives the empty dutyCycleOut object an output
        leftDutycycle.Output = leftOutput;
        // sets the generic controlRequest object to a DutyCycleOut with the defined ouput
        periodicIO.leftOutput = leftDutycycle;

        periodicIO.rightOutput = rightDutycycle;
        rightDutycycle.Output = rightOutput;

        Logger.recordOutput("leftOut", leftOutput);
        Logger.recordOutput("right output", rightOutput);
    }

    public void calibrateGyro() {
        for (int i = 0; i < 1; i++) {
            this.gyro.calibrate();
        }
    }

    public void zeroGyro() {
        this.gyro.reset();
    }

    @Override
    public void writePeriodicOutputs() {
        // setControl() takes a controlRequest Object and applies it's output to the motor
        this.left.setControl(periodicIO.leftOutput);
        this.right.setControl(periodicIO.rightOutput);
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.inputs.leftVel = this.left.getVelocity().getValue();
        periodicIO.inputs.rightVel = this.left.getVelocity().getValue();
        periodicIO.inputs.gyroAngle = this.gyro.getAngle(IMUAxis.kYaw);
        periodicIO.inputs.odoPose =
                this.odo.update(
                        Rotation2d.fromDegrees(periodicIO.inputs.gyroAngle),
                        new DifferentialDriveWheelPositions(getLeftMeters(), getRightMeters()));

        Logger.processInputs("Drive/Inputs", periodicIO.inputs);
    }

    public double getRightMeters() {
        return this.left.getPosition().getValue()
                * DriveTrainConstants.driveGearRatio
                * DriveTrainConstants.kWheelRadius
                * 2
                * Math.PI;
    }

    public double getLeftMeters() {
        return this.right.getPosition().getValue()
                * DriveTrainConstants.driveGearRatio
                * DriveTrainConstants.kWheelRadius
                * 2
                * Math.PI;
    }

    public static class PeriodicIO {
        public double feedforward = 1;
        public double nominalVoltage = 12;
        // generic ControlRequest, so we can make it specific to control mode later
        public ControlRequest leftOutput;
        public ControlRequest rightOutput;

        public Inputs inputs = new Inputs();

        public class Inputs implements LoggableInputs {
            @Override
            public void toLog(LogTable table) {
                table.put("rightVel", rightVel);
                table.put("leftvel", leftVel);
                table.put("gyroAngle", gyroAngle);
                table.put("odo pose", odoPose);
            }

            @Override
            public void fromLog(LogTable table) {
                rightVel = table.get("rightVel", -1);
                leftVel = table.get("leftvel", -1);
                gyroAngle = table.get("gyroAngle", -1);
                odoPose = table.get("odo pose", new Pose2d(-1,-1,Rotation2d.fromDegrees(-1)));
            }

            public double leftVel = 0;
            public double rightVel = 0;

            public double gyroAngle = 0;

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
