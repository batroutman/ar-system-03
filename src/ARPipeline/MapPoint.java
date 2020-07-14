package ARPipeline;

import org.opencv.core.Mat;

public class MapPoint {

	// estimated world coordinates
	protected Double x = null;
	protected Double y = null;
	protected Double z = null;
	
	// observed projected coordinates
	protected Double u = null;
	protected Double v = null;
	
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
	
	public void setUV(Double u, Double v) {
		this.u = u;
		this.v = v;
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

	public Double getU() {
		return u;
	}

	public void setU(Double u) {
		this.u = u;
	}

	public Double getV() {
		return v;
	}

	public void setV(Double v) {
		this.v = v;
	}
	
}
