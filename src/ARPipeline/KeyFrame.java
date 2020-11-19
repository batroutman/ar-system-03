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
	protected ArrayList<Mat> descriptors = new ArrayList<Mat>();
	protected ArrayList<Point2D> keypoints = new ArrayList<Point2D>();

	// list of active search data for every feature being tracked. List length
	// is equal to number of features in this keyframe
	protected ArrayList<ActiveSearchData> searchData = new ArrayList<ActiveSearchData>();

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

	public ArrayList<Correspondence2D2D> getCorrespondences(Frame frame, Frame outputFrame, long frameNum) {
		ArrayList<Correspondence2D2D> correspondences = new ArrayList<Correspondence2D2D>();

		final int FLICKER_BUFFER = 10;

		// get all features in the image
		MatOfKeyPoint keypointsFound = new MatOfKeyPoint();
		Mat descriptorsFound = new Mat();
		ARUtils.fullFrameFeatureDetect(frame, outputFrame, keypointsFound, descriptorsFound, 40, false);

		// load features into hashmaps
		java.util.HashMap<String, ArrayList<KeyPoint>> keypointMap = new java.util.HashMap<String, ArrayList<KeyPoint>>();
		java.util.HashMap<String, ArrayList<Mat>> descriptorMap = new java.util.HashMap<String, ArrayList<Mat>>();
		List<KeyPoint> keypointList = keypointsFound.toList();
		for (int i = 0; i < keypointList.size(); i++) {

			KeyPoint kp = keypointList.get(i);

			if (keypointMap.get((int) kp.pt.x + "," + (int) kp.pt.y) == null) {
				keypointMap.put((int) kp.pt.x + "," + (int) kp.pt.y, new ArrayList<KeyPoint>());
				descriptorMap.put((int) kp.pt.x + "," + (int) kp.pt.y, new ArrayList<Mat>());
			}
			ArrayList<KeyPoint> keypointsInMap = keypointMap.get((int) kp.pt.x + "," + (int) kp.pt.y);
			ArrayList<Mat> descriptorsInMap = descriptorMap.get((int) kp.pt.x + "," + (int) kp.pt.y);

			keypointsInMap.add(kp);
			descriptorsInMap.add(descriptorsFound.row(i));

		}

		// for every feature in the keyframe
		for (int i = 0; i < this.searchData.size(); i++) {
			// if tracking has been lost on this feature, skip
			if (this.searchData.get(i).getLastFrameObserved() + FLICKER_BUFFER < frameNum)
				continue;

			// calculate predicted position based on constant velocity 2D motion
			// of feature
			long framesSinceSeen = frameNum - this.searchData.get(i).getLastFrameObserved();
			int targetX = (int) (this.searchData.get(i).getLastLocation().getX()
					+ this.searchData.get(i).getDx() * framesSinceSeen);
			int targetY = (int) (this.searchData.get(i).getLastLocation().getY()
					+ this.searchData.get(i).getDy() * framesSinceSeen);

			int WINDOW_SIZE = this.searchData.get(i).getWindowSize();

			ArrayList<KeyPoint> windowKeypoints = new ArrayList<KeyPoint>();
			ArrayList<Mat> windowDescriptors = new ArrayList<Mat>();
			ARUtils.getFeaturesInWindow(frame, outputFrame, targetX, targetY, WINDOW_SIZE, keypointMap, descriptorMap,
					windowKeypoints, windowDescriptors);

			// match these features against last descriptor
			Integer match = ARUtils.matchDescriptor(this.searchData.get(i).getLastDescriptor(), windowKeypoints,
					windowDescriptors);

			// if the feature was not found, increase the window size and try
			// again. Otherwise, decrease the window size
			if (match == null) {
				// this.searchData.get(i).increaseWindowSize();
				WINDOW_SIZE = ActiveSearchData.LARGE_BOX;
				// WINDOW_SIZE = this.searchData.get(i).getWindowSize();
				ARUtils.getFeaturesInWindow(frame, outputFrame, targetX, targetY, WINDOW_SIZE, keypointMap,
						descriptorMap, windowKeypoints, windowDescriptors);
				match = ARUtils.matchDescriptor(this.searchData.get(i).getLastDescriptor(), windowKeypoints,
						windowDescriptors, 0.25, 20);
			} else {
				// this.searchData.get(i).decreaseWindowSize();
			}

			// if there is a good match, generate correspondence and update
			// lastKeypointLocations, lastKeypointDescriptors, and
			// lastFrameObserved
			if (match != null) {
				this.searchData.get(i).setLastFrameObserved(frameNum);
				this.searchData.get(i).setLastDescriptor(windowDescriptors.get(match));

				Point2D point = new Point2D();
				point.setX(windowKeypoints.get(match).pt.x);
				point.setY(windowKeypoints.get(match).pt.y);

				this.searchData.get(i)
						.registerDx((point.getX() - this.searchData.get(i).getLastLocation().getX()) / framesSinceSeen);
				this.searchData.get(i)
						.registerDy((point.getY() - this.searchData.get(i).getLastLocation().getY()) / framesSinceSeen);

				this.searchData.get(i).setLastLocation(point);

				// create correspondence
				Correspondence2D2D c = new Correspondence2D2D();
				c.setU1(this.keypoints.get(i).getX());
				c.setV1(this.keypoints.get(i).getY());
				c.setU2(point.getX());
				c.setV2(point.getY());
				c.setDescriptor1(this.descriptors.get(i));
				c.setDescriptor2(windowDescriptors.get(match));
				c.setDu(this.searchData.get(i).getDx());
				c.setDv(this.searchData.get(i).getDy());
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

	public ArrayList<Mat> getDescriptors() {
		return descriptors;
	}

	public void setDescriptors(ArrayList<Mat> descriptors) {
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

	public ArrayList<ActiveSearchData> getSearchData() {
		return searchData;
	}

	public void setSearchData(ArrayList<ActiveSearchData> searchData) {
		this.searchData = searchData;
	}

}
