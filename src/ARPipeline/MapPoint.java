package ARPipeline;

import org.opencv.core.Mat;

public class MapPoint {

	// estimated world coordinates
	protected Double x = null;
	protected Double y = null;
	protected Double z = null;
	
	
	// initial descriptor
	protected Mat descriptor = null;
	
	public MapPoint() {
		
	}
	
	public MapPoint(Mat descriptor) {
		this.descriptor = descriptor;
	}
	
	public void setPosition(Double x, Double y, Double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public Double getX() {
		return x;
	}

	public void setX(Double x) {
		this.x = x;
	}

	public Double getY() {
		return y;
	}

	public void setY(Double y) {
		this.y = y;
	}

	public Double getZ() {
		return z;
	}

	public void setZ(Double z) {
		this.z = z;
	}

	public Mat getDescriptor() {
		return descriptor;
	}

	public void setDescriptor(Mat descriptor) {
		this.descriptor = descriptor;
	}
	
}
