package ARPipeline;

import org.opencv.core.Mat;

public class KeyFrame {

	protected MapPointKDTree descriptors = null;
	protected Pose pose = null;
	
	protected Mat descriptorsMat = null;
	
	public KeyFrame() {
		
	}
	
	public KeyFrame(MapPointKDTree descriptors) {
		this.descriptors = descriptors;
	}

	public MapPointKDTree getDescriptors() {
		return descriptors;
	}

	public void setDescriptors(MapPointKDTree descriptors) {
		this.descriptors = descriptors;
	}

	public Pose getPose() {
		return pose;
	}

	public void setPose(Pose pose) {
		this.pose = pose;
	}

	public Mat getDescriptorsMat() {
		return descriptorsMat;
	}

	public void setDescriptorsMat(Mat descriptorsMat) {
		this.descriptorsMat = descriptorsMat;
	}
	
}
