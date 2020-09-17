package ARPipeline;

import org.opencv.core.Mat;

public class KeyFrameEntry {

	protected Point3D point = null;
	protected Mat descriptor = null;
	protected Point2D keypoint = null;

	public KeyFrameEntry() {

	}

	public Point3D getPoint() {
		return point;
	}

	public void setPoint(Point3D point) {
		this.point = point;
	}

	public Mat getDescriptor() {
		return descriptor;
	}

	public void setDescriptor(Mat descriptor) {
		this.descriptor = descriptor;
	}

	public Point2D getKeypoint() {
		return keypoint;
	}

	public void setKeypoint(Point2D keypoint) {
		this.keypoint = keypoint;
	}

}
