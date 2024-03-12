package team3647.frc2024.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import team3647.lib.GeomUtil;

public class Poton extends PhotonCamera implements AprilTagCamera {
    private final Supplier<Pose2d> odoPose;
    private final AprilTagFieldLayout layout =
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private final PhotonPoseEstimator photonPoseEstimator;
    private final Matrix<N3, N1> baseStdDevs;

    public Poton(String name, Supplier<Pose2d> odoPose) {
        super(name);
        this.odoPose = odoPose;
        this.photonPoseEstimator =
                new PhotonPoseEstimator(
                        layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d());
        this.baseStdDevs = VecBuilder.fill(0.05, 0.05, 0.05);
    }

    @Override
    public Optional<VisionData> queueToInputs() {
        var result = super.getLatestResult();
        if (!result.hasTargets()) {
            return Optional.empty();
        }

        var botPose = photonPoseEstimator.update(result);
        if (botPose.isEmpty()) {
            return Optional.empty();
        }
        Pose3d realPose = botPose.get().estimatedPose;

        double stdDevsScalar =
                getDistToTag(realPose, getTagNum()) + result.getTargets().size() * 100;
        var newStDevs = baseStdDevs.times(stdDevsScalar);

        double timestamp = result.getTimestampSeconds();

        var data = new VisionData(realPose.toPose2d(), timestamp, newStDevs);
        return Optional.of(data);
    }

    public double getDistToTag(Pose3d visionPose, int tagID) {
        Optional<Pose3d> tagposeMabye = layout.getTagPose((int) (tagID));

        Pose3d tagpose = new Pose3d();

        if (tagposeMabye.isPresent()) {
            tagpose = tagposeMabye.get();
        }
        double distFromTag = GeomUtil.distance(visionPose.toPose2d(), tagpose.toPose2d());
        return distFromTag;
    }

    @Override
    public int getTagNum() {
        return super.getLatestResult().getBestTarget().getFiducialId();
    }
}
