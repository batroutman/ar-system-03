package ARPipeline;

public interface PoseBuffer {

	public void pushPose(double r00, double r01, double r02, double r10, double r11, double r12, double r20, double r21, double r22, double t0, double t1, double t2, double timestamp);

}