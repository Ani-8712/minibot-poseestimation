package team3647.frc2024.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionData {
    public Pose2d pose;
    public double timestamp;
    public Matrix<N3, N1> stdDevs;

    public VisionData(Pose2d pose, double timestamp, Matrix<N3, N1> sdtDevs) {
        this.pose = pose;
        this.timestamp = timestamp;
        this.stdDevs = sdtDevs;
    }
}
