package team3647.frc2024.util;

import java.util.Optional;

/** AprilTagCamera */
public interface AprilTagCamera {
    public Optional<VisionMeasurement> queueToInputs();

    public int getTagNum();
}
