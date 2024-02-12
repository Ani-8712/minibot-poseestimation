package team3647.frc2024.util;

import java.util.Optional;

/** AprilTagCamera */
public interface AprilTagCamera {
    public Optional<VisionData> queueToInputs();

    public int getTagNum();
}
