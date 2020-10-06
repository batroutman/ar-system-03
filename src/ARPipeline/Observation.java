package ARPipeline;

import org.opencv.core.Mat;

public class Observation {

	protected KeyFrame keyframe = null;
	protected Mat descriptor = null;
	protected Point2D point = null;

	public Observation() {

	}

	public Observation(KeyFrame keyframe, Mat descriptor, Point2D point) {
		this.keyframe = keyframe;
		this.descriptor = descriptor;
		this.point = point;
	}

	public KeyFrame getKeyframe() {
		return keyframe;
	}

	public void setKeyframe(KeyFrame keyframe) {
		this.keyframe = keyframe;
	}

	public Mat getDescriptor() {
		return descriptor;
	}

	public void setDescriptor(Mat descriptor) {
		this.descriptor = descriptor;
	}

	public Point2D getPoint() {
		return point;
	}

	public void setPoint(Point2D point) {
		this.point = point;
	}

}
