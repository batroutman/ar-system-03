package ARPipeline;

public interface PoseBuffer {

	public void pushPose(Pose pose);
	public Pose getCurrentPose();

}