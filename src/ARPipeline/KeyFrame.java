package ARPipeline;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;

public class KeyFrame {

	// pose with respect to origin/identity
	protected Pose pose = null;

	protected ArrayList<MapPoint> mapPoints = new ArrayList<MapPoint>();
	protected Mat descriptors = null;
	protected ArrayList<Point2D> keypoints = new ArrayList<Point2D>();
	protected ArrayList<Point2D> lastKeypointLocations = new ArrayList<Point2D>();
	protected ArrayList<Mat> lastKeypointDescriptors = new ArrayList<Mat>();
	protected ArrayList<Long> lastFrameObserved = new ArrayList<Long>();
	protected long frameNumber = 0;

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

	public ArrayList<Correspondence2D2D> getCorrespondences(Frame frame, long frameNum) {
		ArrayList<Correspondence2D2D> correspondences = new ArrayList<Correspondence2D2D>();

		final int FLICKER_BUFFER = 10;
		final int WINDOW_SIZE = 10;

		Mat image = ARUtils.frameToMat(frame);

		// for every feature in the keyframe
		for (int i = 0; i < this.getLastKeypointLocations().size(); i++) {
			// if tracking has been lost on this feature, skip
			if (this.lastFrameObserved.get(i) + FLICKER_BUFFER < frameNum)
				continue;

			// get features in window around last keypoint
			MatOfKeyPoint keypointsFound = new MatOfKeyPoint();
			Mat descriptorsFound = new Mat();

			long start = System.currentTimeMillis();
			ARUtils.getFeaturesInWindow(frame, image, (int) this.lastKeypointLocations.get(i).getX(),
					(int) this.lastKeypointLocations.get(i).getY(), WINDOW_SIZE, keypointsFound, descriptorsFound);
			long end = System.currentTimeMillis();
			// System.out.println("getFeaturesInWindow: " + (end - start) + "
			// ms");

			// match these features against last descriptor
			Integer match = ARUtils.matchDescriptor(this.lastKeypointDescriptors.get(i), keypointsFound,
					descriptorsFound);

			// if there is a good match, generate correspondence and update
			// lastKeypointLocations, lastKeypointDescriptors, and
			// lastFrameObserved
			if (match != null) {
				this.lastFrameObserved.set(i, frameNum);
				this.lastKeypointDescriptors.set(i, descriptorsFound.row(match));

				List<KeyPoint> keypointList = keypointsFound.toList();
				Point2D point = new Point2D();
				point.setX(keypointList.get(match).pt.x);
				point.setY(keypointList.get(match).pt.y);
				this.lastKeypointLocations.set(i, point);

				// create correspondence
				Correspondence2D2D c = new Correspondence2D2D();
				c.setU1(this.keypoints.get(i).getX());
				c.setV1(this.keypoints.get(i).getY());
				c.setU2(point.getX());
				c.setV2(point.getY());
				correspondences.add(c);
			}
		}

		return correspondences;
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

	public long getFrameNumber() {
		return frameNumber;
	}

	public void setFrameNumber(long frameNumber) {
		this.frameNumber = frameNumber;
	}

	public ArrayList<Point2D> getLastKeypointLocations() {
		return lastKeypointLocations;
	}

	public void setLastKeypointLocations(ArrayList<Point2D> lastKeypointLocations) {
		this.lastKeypointLocations = lastKeypointLocations;
	}

	public ArrayList<Mat> getLastKeypointDescriptors() {
		return lastKeypointDescriptors;
	}

	public void setLastKeypointDescriptors(ArrayList<Mat> lastKeypointDescriptors) {
		this.lastKeypointDescriptors = lastKeypointDescriptors;
	}

	public ArrayList<Long> getLastFrameObserved() {
		return lastFrameObserved;
	}

	public void setLastFrameObserved(ArrayList<Long> lastFrameObserved) {
		this.lastFrameObserved = lastFrameObserved;
	}

}
