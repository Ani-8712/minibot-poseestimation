package team3647.frc2024.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
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

    public Limelight(String limelightName, Supplier<Pose2d> odoPose) {
        this.name = limelightName;
        this.baseStdDevs = VecBuilder.fill(0.05, 0.05, 0.05);
        this.odoPose = odoPose;
    }

    @Override
    public Optional<VisionData> queueToInputs() {
        var result = LimelightHelpers.getBotPose2d(name);

        double timestamp = Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Pipeline(name);
        double distFromTag = GeomUtil.distance(result, odoPose.get());
        double stdDevsScalar = distFromTag + getNumberOfTargets() * 100;
        var stdDevs = baseStdDevs.times(stdDevsScalar);

        VisionData data = new VisionData(result, timestamp, stdDevs);
        return Optional.of(data);
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
