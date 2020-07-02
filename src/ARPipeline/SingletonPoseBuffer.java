package ARPipeline;

public class SingletonPoseBuffer implements PoseBuffer{
	
	Pose pose;

	public SingletonPoseBuffer() {
		
	}
	
	public void pushPose(Pose pose) {
		this.pose = pose;
	}
	
	public Pose getCurrentPose() {
		return this.pose;
	}
	
}
