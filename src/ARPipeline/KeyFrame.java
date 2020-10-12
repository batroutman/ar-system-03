package ARPipeline;

import java.util.ArrayList;

import org.opencv.core.Mat;

public class KeyFrame {

	// pose with respect to origin/identity
	protected Pose pose = null;

	protected ArrayList<MapPoint> mapPoints = new ArrayList<MapPoint>();
	protected Mat descriptors = null;
	protected ArrayList<Point2D> keypoints = new ArrayList<Point2D>();

	// hashtable that will take a point observation in comma-separated string
	// form (for example,
	// "15,489") and return the corresponding MapPoint generated for that
	// observation
	protected java.util.Map<String, MapPoint> obsvToMapPoint = new java.util.HashMap<String, MapPoint>();

	public KeyFrame() {

	}

	public KeyFrame(ArrayList<MapPoint> mapPoints) {
		this.mapPoints = mapPoints;
	}

	public Pose getPose() {
		return pose;
	}

	public void setPose(Pose pose) {
		this.pose = pose;
	}

	public ArrayList<MapPoint> getMapPoints() {
		return mapPoints;
	}

	public void setMapPoints(ArrayList<MapPoint> mapPoints) {
		this.mapPoints = mapPoints;
	}

	public Mat getDescriptors() {
		return descriptors;
	}

	public void setDescriptors(Mat descriptors) {
		this.descriptors = descriptors;
	}

	public ArrayList<Point2D> getKeypoints() {
		return keypoints;
	}

	public void setKeypoints(ArrayList<Point2D> keypoints) {
		this.keypoints = keypoints;
	}

	public java.util.Map<String, MapPoint> getObsvToMapPoint() {
		return obsvToMapPoint;
	}

	public void setObsvToMapPoint(java.util.Map<String, MapPoint> obsvToMapPoint) {
		this.obsvToMapPoint = obsvToMapPoint;
	}

}
