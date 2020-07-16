package ARPipeline;

import org.opencv.core.Mat;

public class Correspondence {
	
	MapPoint mapPoint = null;
	
	double lastFrameU;
	double lastFrameV;
	Mat lastFrameDescriptor = null;

	public Correspondence() {
		
	}

	public MapPoint getMapPoint() {
		return mapPoint;
	}

	public void setMapPoint(MapPoint mapPoint) {
		this.mapPoint = mapPoint;
	}

	public double getLastFrameU() {
		return lastFrameU;
	}

	public void setLastFrameU(double lastFrameU) {
		this.lastFrameU = lastFrameU;
	}

	public double getLastFrameV() {
		return lastFrameV;
	}

	public void setLastFrameV(double lastFrameV) {
		this.lastFrameV = lastFrameV;
	}

	public Mat getLastFrameDescriptor() {
		return lastFrameDescriptor;
	}

	public void setLastFrameDescriptor(Mat lastFrameDescriptor) {
		this.lastFrameDescriptor = lastFrameDescriptor;
	}

}
