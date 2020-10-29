package ARPipeline;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;

import Jama.Matrix;

public class Map {

	protected ArrayList<MapPoint> mapPoints = new ArrayList<MapPoint>();
	protected ArrayList<KeyFrame> keyframes = new ArrayList<KeyFrame>();

	public Map() {

	}

	public KeyFrame generateInitialKeyFrame(Mat descriptors, MatOfKeyPoint keypoints, long frameNum) {
		List<KeyPoint> keypointList = keypoints.toList();
		KeyFrame keyframe = new KeyFrame();
		keyframe.setFrameNumber(frameNum);
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
			this.mapPoints.add(mp);
		}
		this.keyframes.add(keyframe);
		return keyframe;
	}

	public KeyFrame registerNewKeyframe(Mat descriptors, MatOfKeyPoint keypoints, long frameNum, Pose currentPose,
			ArrayList<Correspondence2D2D> correspondences, ArrayList<MapPoint> existingMapPoints) {
		List<KeyPoint> keypointList = keypoints.toList();
		KeyFrame keyframe = new KeyFrame();
		keyframe.setFrameNumber(frameNum);

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
			this.mapPoints.add(mp);
		}
		this.keyframes.add(keyframe);
		return keyframe;
	}

	// given a camera pose, the camera intrinsics, and the width and height of
	// the frame, generate a visualization of the map's currently triangulated
	// map points
	public Frame getMapVisualizationFrame(Pose pose, Matrix K, int width, int height) {
		byte[] r = new byte[width * height];
		byte[] g = new byte[width * height];
		byte[] b = new byte[width * height];

		byte[][] colors = { { (byte) 255, 0, 0 }, { 0, (byte) 255, 0 }, { 0, 0, (byte) 255 },
				{ (byte) 255, (byte) 255, 0 }, { (byte) 255, 0, (byte) 255 }, { 0, (byte) 255, (byte) 255 } };

		for (int i = 0; i < width * height; i++) {
			r[i] = 0;
			g[i] = 0;
			b[i] = 0;
		}
		for (MapPoint mapPoint : this.mapPoints) {
			if (mapPoint.getPoint() == null) {
				continue;
			}
			Point3D point3D = mapPoint.getPoint();
			Matrix projectedPoint = K.times(pose.getHomogeneousMatrix().getMatrix(0, 2, 0, 3))
					.times(point3D.getHomogeneousPoint());
			double w = projectedPoint.get(2, 0);
			projectedPoint = projectedPoint.times(1 / w);
			// if (w < 0) {
			// System.out.println("w = 0 point projected behind camera");
			// }
			// if (projectedPoint.get(1, 0) < 0) {
			// System.out.println("CAMERA TOO LOW: " + projectedPoint.get(1,
			// 0));
			// }
			if (w > 0 && projectedPoint.get(0, 0) >= 0 && projectedPoint.get(0, 0) < width
					&& projectedPoint.get(1, 0) >= 0 && projectedPoint.get(1, 0) < height) {
				byte[] color = colors[Math.min(mapPoint.getObservations().size() - 1, colors.length)];
				r[(int) projectedPoint.get(1, 0) * width + (int) projectedPoint.get(0, 0)] = color[0];
				g[(int) projectedPoint.get(1, 0) * width + (int) projectedPoint.get(0, 0)] = color[1];
				b[(int) projectedPoint.get(1, 0) * width + (int) projectedPoint.get(0, 0)] = color[2];
			}
		}
		Frame frame = new Frame(r, g, b, width, height);
		return frame;
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
