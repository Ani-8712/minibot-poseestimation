package team3647.frc2024.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    private static final AprilTagFieldLayout tags =
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final Pose2d kBlueSpeakerPose = new Pose2d(0, 5.55, Rotation2d.fromDegrees(180));

    public static Pose2d getTagPose(int id){
        var pose = tags.getTagPose(id);
        if(pose.isPresent()){
            return pose.get().toPose2d();
        }else{
            return new Pose2d(-1, -1, Rotation2d.fromDegrees(-1));
        }
    }
}
