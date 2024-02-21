package team3647.frc2024.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team3647.frc2024.Constants.DriveTrainConstants;
import team3647.frc2024.commands.DrivetrainCommands;
import team3647.frc2024.subsystems.Drivetrain;
import team3647.lib.inputs.Joysticks;

/**
 * This is where we define the bindings for all our commands. Subsystems have the built in method
 * setDefaultCommand(). This sets the command that runs on a subsystem when there is no other
 * command that overrides it. Here we use it to set the drive command.
 */
public class RobotContainer {

    private final Drivetrain drivetrain =
            new Drivetrain(
                    DriveTrainConstants.kLeftMotor,
                    DriveTrainConstants.kRightMotor,
                    DriveTrainConstants.gyro,
                    DriveTrainConstants.kinematics);
    // we use the custom Joysticks class for our input, (NOT THE BUILT-IN JOYSTICK CLASS) so we need
    // to copy it from aother robot's code
    private final Joysticks mainController = new Joysticks(0);

    private final DrivetrainCommands drivetrainCommands = new DrivetrainCommands(drivetrain);

    public RobotContainer() {
        // It is necessary to register all subsysems with the command scheduler, so it knows what
        // they are.
        CommandScheduler.getInstance().registerSubsystem(drivetrain);

        drivetrain.setDefaultCommand(
                drivetrainCommands.drive(
                        mainController::getLeftStickY, mainController::getRightStickX));

        drivetrain.calibrateGyro();
        drivetrain.zeroGyro();
        

        mainController.buttonA.whileTrue(drivetrainCommands.shoot(() -> 0.7));
        mainController.buttonX.whileTrue(
                drivetrainCommands.faceSpeaker(DriveTrainConstants.kRotController));
        mainController.buttonY.whileTrue(drivetrainCommands.shoot(() -> 1));

        /**
         * The code below is an example of how to set a command to a button on the controller.
         * Notice how it uses the same subsystem as the default command we set above. When I am
         * pressing A on the controller, the drivetrain drives at 0.1 of its normal speed, but when
         * I let go, that command gets descheduled, and the default commands starts again, so I can
         * continue driving normally.
         *
         * <p>NOTE: the () -> maincontroller.getRightStickX() syntax is functionally equivalent to
         * the maincontoller::getRightStickX, but I can multiply by 0.1 if I use the lambda syntax
         */
        mainController.buttonB.whileTrue(
                drivetrainCommands.drive(
                        () -> mainController.getLeftStickY() * 0.1,
                        () -> mainController.getRightStickX() * 0.1));
    }
}
