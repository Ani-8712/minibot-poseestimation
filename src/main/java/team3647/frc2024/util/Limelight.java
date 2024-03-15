package team3647.frc2024.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.function.Supplier;
import team3647.lib.GeomUtil;
import team3647.lib.LimelightHelpers;

public class Limelight implements AprilTagCamera {
    private final String name;
    private final Matrix<N3, N1> baseStdDevs;
    private final Supplier<Pose2d> odoPose;
    AprilTagFieldLayout layout;

    public Limelight(String limelightName, Supplier<Pose2d> odoPose) {
        this.name = limelightName;
        this.baseStdDevs = VecBuilder.fill(0.05, 0.05, 0.05);
        this.odoPose = odoPose;
        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

    @Override
    public Optional<VisionData> queueToInputs() {
        var result = LimelightHelpers.getBotPose3d_wpiBlue(name);

        if(result.equals(new Pose3d())){
            return Optional.empty();
        }


        if(result.getZ() > 0){
            return Optional.empty();
        }

        double timestamp = Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline(name)) / 1000.0;

        double distFromTag = GeomUtil.distance(result.toPose2d(), getTagPose().toPose2d());
        double stdDevsScalar = distFromTag/getNumberOfTargets() * 100;
        var stdDevs = baseStdDevs
        .times(stdDevsScalar);

        VisionData data = new VisionData(result.toPose2d(), timestamp, stdDevs);
        return Optional.of(data);
    }

    public Pose3d getTagPose() {
        Optional<Pose3d> tagposeMabye =
                layout.getTagPose((int) (LimelightHelpers.getFiducialID(name)));

        Pose3d tagpose = new Pose3d();
        if (tagposeMabye.isPresent()) {
            tagpose = tagposeMabye.get();
        }

        return tagpose;
    }

    public int getNumberOfTargets() {
        var results = LimelightHelpers.getLatestResults(name);
        int num = results.targetingResults.targets_Fiducials.length;
        return num;
    }

    @Override
    public int getTagNum() {
        return (int) LimelightHelpers.getFiducialID(name);
    }
}
