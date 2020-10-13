package ARPipeline;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;

public class Map {

	protected ArrayList<MapPoint> mapPoints = new ArrayList<MapPoint>();
	protected ArrayList<KeyFrame> keyframes = new ArrayList<KeyFrame>();

	public Map() {

	}

	public KeyFrame generateInitialKeyFrame(Mat descriptors, MatOfKeyPoint keypoints) {
		List<KeyPoint> keypointList = keypoints.toList();
		KeyFrame keyframe = new KeyFrame();
		Pose pose = new Pose();
		pose.setFixed(true);
		keyframe.setPose(pose);
		keyframe.setDescriptors(descriptors);
		for (int i = 0; i < keypointList.size(); i++) {
			Point2D point = new Point2D(keypointList.get(i).pt.x, keypointList.get(i).pt.y);
			keyframe.getKeypoints().add(point);
			Observation observation = new Observation();
			observation.setDescriptor(descriptors.row(i));
			observation.setPoint(point);
			observation.setKeyframe(keyframe);
			MapPoint mp = new MapPoint();
			mp.setPrincipalDescriptor(descriptors.row(i));
			mp.getObservations().add(observation);
			keyframe.getMapPoints().add(mp);
			keyframe.getObsvToMapPoint().put((int) point.getX() + "," + (int) point.getY(), mp);
		}
		this.keyframes.add(keyframe);
		return keyframe;
	}

	public KeyFrame registerNewKeyframe(Mat descriptors, MatOfKeyPoint keypoints, Pose currentPose,
			ArrayList<Correspondence2D2D> correspondences, ArrayList<MapPoint> existingMapPoints) {
		List<KeyPoint> keypointList = keypoints.toList();
		KeyFrame keyframe = new KeyFrame();

		// set up new pose
		Pose pose = new Pose();
		pose.setFixed(false);
		pose.setCx(currentPose.getCx());
		pose.setCy(currentPose.getCy());
		pose.setCz(currentPose.getCz());
		pose.setQw(currentPose.getQw());
		pose.setQx(currentPose.getQx());
		pose.setQy(currentPose.getQy());
		pose.setQz(currentPose.getQz());
		keyframe.setPose(pose);
		keyframe.setDescriptors(descriptors);

		// add pre existing map points to hash table
		for (int c = 0; c < correspondences.size(); c++) {
			keyframe.getObsvToMapPoint().put(
					correspondences.get(c).getU2().intValue() + "," + correspondences.get(c).getV2().intValue(),
					existingMapPoints.get(c));
		}

		// go through all keypoints and handle map points
		for (int i = 0; i < keypointList.size(); i++) {
			// check if map point is in hash table
			MapPoint mp = keyframe.getObsvToMapPoint()
					.get((int) keypointList.get(i).pt.x + "," + (int) keypointList.get(i).pt.y);
			if (mp == null) {
				mp = new MapPoint();
				keyframe.getObsvToMapPoint().put((int) keypointList.get(i).pt.x + "," + (int) keypointList.get(i).pt.y,
						mp);
			}

			// update map point
			Point2D point = new Point2D(keypointList.get(i).pt.x, keypointList.get(i).pt.y);
			keyframe.getKeypoints().add(point);
			Observation observation = new Observation();
			observation.setDescriptor(descriptors.row(i));
			observation.setPoint(point);
			observation.setKeyframe(keyframe);
			// mp.setPrincipalDescriptor(descriptors.row(i));
			mp.getObservations().add(observation);
			keyframe.getMapPoints().add(mp);
		}
		this.keyframes.add(keyframe);
		return keyframe;
	}

	public ArrayList<KeyFrame> getKeyframes() {
		return keyframes;
	}

	public void setKeyframes(ArrayList<KeyFrame> keyframes) {
		this.keyframes = keyframes;
	}

	public ArrayList<MapPoint> getMapPoints() {
		return mapPoints;
	}

	public void setMapPoints(ArrayList<MapPoint> mapPoints) {
		this.mapPoints = mapPoints;
	}

}
