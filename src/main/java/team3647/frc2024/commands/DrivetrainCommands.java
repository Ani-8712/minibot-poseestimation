package team3647.frc2024.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import java.util.function.DoubleSupplier;
import team3647.frc2024.subsystems.Drivetrain;

/**
 * We use the commands class to define everything we are *going* to do with the subystem. For
 * example, in a shooter, we define methods to run the rollers in the subsystem, then
 * we use these methods to set specific values and setpoints in the commands. For more info on
 * commands, see https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html
 */
public class DrivetrainCommands {

    private final Drivetrain drivetrain;

    public DrivetrainCommands(Drivetrain drive) {
        this.drivetrain = drive;
    }

    /**
     * doubleSuppliers are functions that return doubles, and their values are continuouisly called.
     * If we used doubles, we would only be able to pass in one value for the whole time the command
     * is active, so the value of the joystick when the command is initialized would be the speed
     * the whole time.
     */
    public Command drive(DoubleSupplier drive, DoubleSupplier turn) {
        return Commands.run(
                () -> {
                    SmartDashboard.putNumber("Drive", drive.getAsDouble());
                    SmartDashboard.putNumber("Turn", turn.getAsDouble());
                    // uses the .drive method from the drivetrain class; multiplying by 0.5 makes it
                    // run at half sped
                    drivetrain.drive(drive.getAsDouble() * 0.5, turn.getAsDouble() * 0.5, false);
                },
                drivetrain);
    }

    public Command faceSpeaker(PIDController rotController) {
        return new Command() {
            double angle;

            @Override
            public void initialize() {
                angle = drivetrain.getAngleToSpeaker();
            }

            @Override
            public void execute() {

                var demand = rotController.calculate(angle - drivetrain.getOdoRot());
                drivetrain.drive(0, demand, false);
                // Logger.recordOutput("pid output", demand);
                // Logger.recordOutput("angle pose2d", new
                // Pose2d(drivetrain.getOdoPose().getTranslation(),
                //         Rotation2d.fromDegrees(angle)));
                // Logger.recordOutput("angle tospeaker", angle);
                // Logger.recordOutput("atGoal?", rotController.atSetpoint());
                // Logger.recordOutput("angle", angle);

            }

            @Override
            public boolean isFinished() {
                return false;
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(drivetrain);
            }
        };
    }

    public Command faceAngle(double angle, ProfiledPIDController controller) {
        return Commands.run(
                () -> {
                    var rallingralls = controller.calculate(angle - drivetrain.getOdoRot());
                    drivetrain.drive(0, rallingralls, false);
                },
                drivetrain);
    }

    // public Command faceSpeaker(Profile   dPIDController rotController) {
    // var angle = drivetrain.getAngleToSpeaker();
    // rotController.setTolerance(10);
    // return Commands.run(
    // () -> {
    // var rallingralls = rotController.calculate(angle-drivetrain.getOdoRot());
    // drivetrain.drive(
    // 0, rallingralls , false);
    // SmartDashboard.putNumber("pid output", rallingralls);
    // Logger.recordOutput("angle pose2d",new
    // Pose2d(drivetrain.getOdoPose().getTranslation(),
    // Rotation2d.fromDegrees(angle)));

    // },
    // drivetrain)
    // .until(() -> Math.abs(drivetrain.getYaw() - angle) < 10);
    // }

    public Command shoot(DoubleSupplier speed) {
        return Commands.run(
                () -> drivetrain.setOpenloop(speed.getAsDouble(), -speed.getAsDouble()),
                drivetrain);
    }

    public Command intakeHandoff(DoubleSupplier output) {
        return Commands.run(
                () -> drivetrain.setOpenloop(output.getAsDouble(), output.getAsDouble()),
                drivetrain);
    }
}
