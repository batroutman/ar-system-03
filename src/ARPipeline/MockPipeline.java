package ARPipeline;

import java.util.ArrayList;

import org.ejml.data.DMatrixRMaj;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.features2d.FlannBasedMatcher;

import Jama.Matrix;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.so.Quaternion_F64;

public class MockPipeline extends ARPipeline {

	long lastTime = System.nanoTime();

	MockPointData mock = new MockPointData();
	long frameNum = 0;
	boolean mapInitialized = false;

	Matrix K = new Matrix(3, 3);

	KeyFrame currentKeyFrame = null;

	Map map = new Map();

	Pose pose = new Pose();

	protected Thread mainThread = new Thread() {
		@Override
		public void run() {
			mainloop();
		}
	};

	public MockPipeline() {
		super();
		this.init();
	}

	public MockPipeline(FrameBuffer inputFrameBuffer) {
		super();
		this.init();
		this.setInputFrameBuffer(inputFrameBuffer);
	}

	public MockPipeline(FrameBuffer inputFrameBuffer, PoseBuffer outputPoseBuffer) {
		super();
		this.init();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputPoseBuffer(outputPoseBuffer);
	}

	public MockPipeline(FrameBuffer inputFrameBuffer, FrameBuffer outputFrameBuffer) {
		super();
		this.init();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputFrameBuffer(outputFrameBuffer);
	}

	public MockPipeline(FrameBuffer inputFrameBuffer, PoseBuffer outputPoseBuffer, FrameBuffer outputFrameBuffer) {
		super();
		this.init();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputPoseBuffer(outputPoseBuffer);
		this.setOutputFrameBuffer(outputFrameBuffer);
	}

	public void init() {
		K.set(0, 0, CameraIntrinsics.fx);
		K.set(0, 1, CameraIntrinsics.s);
		K.set(0, 2, CameraIntrinsics.cx);
		K.set(1, 0, 0.0);
		K.set(1, 1, CameraIntrinsics.fy);
		K.set(1, 2, CameraIntrinsics.cy);
		K.set(2, 0, 0.0);
		K.set(2, 1, 0.0);
		K.set(2, 2, 1.0);

	}

	public void start() {
		this.mainThread.start();
	}

	public void stop() {
		this.mainThread.interrupt();
	}

	public void updatePoseFromCurrent(Matrix E) {

		Matrix currentTransform = this.currentKeyFrame.getPose().getHomogeneousMatrix();
		Matrix newTransform = E.times(currentTransform);

		DMatrixRMaj R = new DMatrixRMaj(3, 3);
		R.add(0, 0, newTransform.get(0, 0));
		R.add(0, 1, newTransform.get(0, 1));
		R.add(0, 2, newTransform.get(0, 2));
		R.add(1, 0, newTransform.get(1, 0));
		R.add(1, 1, newTransform.get(1, 1));
		R.add(1, 2, newTransform.get(1, 2));
		R.add(2, 0, newTransform.get(2, 0));
		R.add(2, 1, newTransform.get(2, 1));
		R.add(2, 2, newTransform.get(2, 2));

		Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(R, null);
		q.normalize();

		this.pose.setQw(q.w);
		this.pose.setQx(q.x);
		this.pose.setQy(q.y);
		this.pose.setQz(q.z);

		this.pose.setT(newTransform.get(0, 3), newTransform.get(1, 3), newTransform.get(2, 3));

	}

	public Pose updateTemporaryPoseFromCurrent(Matrix E) {

		Pose tempPose = new Pose();

		Matrix currentTransform = this.currentKeyFrame.getPose().getHomogeneousMatrix();
		Matrix newTransform = E.times(currentTransform);

		DMatrixRMaj R = new DMatrixRMaj(3, 3);
		R.add(0, 0, newTransform.get(0, 0));
		R.add(0, 1, newTransform.get(0, 1));
		R.add(0, 2, newTransform.get(0, 2));
		R.add(1, 0, newTransform.get(1, 0));
		R.add(1, 1, newTransform.get(1, 1));
		R.add(1, 2, newTransform.get(1, 2));
		R.add(2, 0, newTransform.get(2, 0));
		R.add(2, 1, newTransform.get(2, 1));
		R.add(2, 2, newTransform.get(2, 2));

		Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(R, null);
		q.normalize();

		tempPose.setQw(q.w);
		tempPose.setQx(q.x);
		tempPose.setQy(q.y);
		tempPose.setQz(q.z);

		tempPose.setT(newTransform.get(0, 3), newTransform.get(1, 3), newTransform.get(2, 3));

		return tempPose;

	}

	public void setPose(Matrix E) {

		DMatrixRMaj R = new DMatrixRMaj(3, 3);
		R.add(0, 0, E.get(0, 0));
		R.add(0, 1, E.get(0, 1));
		R.add(0, 2, E.get(0, 2));
		R.add(1, 0, E.get(1, 0));
		R.add(1, 1, E.get(1, 1));
		R.add(1, 2, E.get(1, 2));
		R.add(2, 0, E.get(2, 0));
		R.add(2, 1, E.get(2, 1));
		R.add(2, 2, E.get(2, 2));

		Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(R, null);
		q.normalize();

		this.pose.setQw(q.w);
		this.pose.setQx(q.x);
		this.pose.setQy(q.y);
		this.pose.setQz(q.z);

		this.pose.setT(E.get(0, 3), E.get(1, 3), E.get(2, 3));
		pl("E (setPose()): ");
		this.pose.getHomogeneousMatrix().print(15, 5);

	}

	public Pose setTemporaryPose(Matrix E) {

		DMatrixRMaj R = new DMatrixRMaj(3, 3);
		R.add(0, 0, E.get(0, 0));
		R.add(0, 1, E.get(0, 1));
		R.add(0, 2, E.get(0, 2));
		R.add(1, 0, E.get(1, 0));
		R.add(1, 1, E.get(1, 1));
		R.add(1, 2, E.get(1, 2));
		R.add(2, 0, E.get(2, 0));
		R.add(2, 1, E.get(2, 1));
		R.add(2, 2, E.get(2, 2));

		Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(R, null);
		q.normalize();

		Pose tempPose = new Pose();

		tempPose.setQw(q.w);
		tempPose.setQx(q.x);
		tempPose.setQy(q.y);
		tempPose.setQz(q.z);

		tempPose.setT(E.get(0, 3), E.get(1, 3), E.get(2, 3));

		return tempPose;

	}

	public void deepReplacePose(Pose newPose) {
		this.pose.setCx(newPose.getCx());
		this.pose.setCy(newPose.getCy());
		this.pose.setCz(newPose.getCz());
		this.pose.setQw(newPose.getQw());
		this.pose.setQx(newPose.getQx());
		this.pose.setQy(newPose.getQy());
		this.pose.setQz(newPose.getQz());
	}

	public static void matchDescriptors(Mat descriptors, MatOfKeyPoint keypoints, KeyFrame currentKeyFrame,
			ArrayList<Correspondence2D2D> correspondences, ArrayList<Point> matchedKeyframePoints,
			ArrayList<Point> matchedPoints) {
		double DIST_THRESH = 120;
		FlannBasedMatcher flann = FlannBasedMatcher.create();
		ArrayList<MatOfDMatch> matches = new ArrayList<MatOfDMatch>();
		flann.knnMatch(descriptors, currentKeyFrame.getDescriptors(), matches, 1);
		for (int i = 0; i < matches.size(); i++) {
			DMatch match = matches.get(i).toList().get(0);
			if (match.distance < DIST_THRESH) {
				Correspondence2D2D c = new Correspondence2D2D();
				c.setU1(currentKeyFrame.getKeypoints().get(match.trainIdx).getX());
				c.setV1(currentKeyFrame.getKeypoints().get(match.trainIdx).getY());
				c.setU2(keypoints.toList().get(match.queryIdx).pt.x);
				c.setV2(keypoints.toList().get(match.queryIdx).pt.y);
				c.setDescriptor1(currentKeyFrame.getDescriptors().row(match.trainIdx));
				c.setDescriptor2(descriptors.row(match.queryIdx));
				correspondences.add(c);
				Point pt = new Point();
				pt.x = currentKeyFrame.getKeypoints().get(match.trainIdx).getX();
				pt.y = currentKeyFrame.getKeypoints().get(match.trainIdx).getY();
				matchedKeyframePoints.add(pt);
				matchedPoints.add(keypoints.toList().get(match.queryIdx).pt);
			}
		}
	}

	public Pose structureFromMotionUpdate(ArrayList<Point> matchedKeyframePoints, ArrayList<Point> matchedPoints,
			ArrayList<Correspondence2D2D> correspondences) {
		// compute fundamental matrix -> essential matrix -> [ R t ]
		MatOfPoint2f keyframeMat = new MatOfPoint2f();
		MatOfPoint2f matKeypoints = new MatOfPoint2f();
		keyframeMat.fromList(matchedKeyframePoints);
		matKeypoints.fromList(matchedPoints);
		Mat fundamentalMatrix = Calib3d.findFundamentalMat(keyframeMat, matKeypoints, Calib3d.FM_8POINT, 0.1, 0.99);

		Matrix funMat = new Matrix(3, 3);
		funMat.set(0, 0, fundamentalMatrix.get(0, 0)[0]);
		funMat.set(0, 1, fundamentalMatrix.get(0, 1)[0]);
		funMat.set(0, 2, fundamentalMatrix.get(0, 2)[0]);
		funMat.set(1, 0, fundamentalMatrix.get(1, 0)[0]);
		funMat.set(1, 1, fundamentalMatrix.get(1, 1)[0]);
		funMat.set(1, 2, fundamentalMatrix.get(1, 2)[0]);
		funMat.set(2, 0, fundamentalMatrix.get(2, 0)[0]);
		funMat.set(2, 1, fundamentalMatrix.get(2, 1)[0]);
		funMat.set(2, 2, fundamentalMatrix.get(2, 2)[0]);
		pl("estimated fundamental matrix:");
		funMat.print(15, 5);

		Matrix eMatrix = this.K.transpose().times(funMat).times(this.K);

		EssentialDecomposition decomp = ARUtils.decomposeEssentialMat(eMatrix);

		Rt rt = ARUtils.selectEssentialSolution(decomp, this.currentKeyFrame.getPose().getHomogeneousMatrix(),
				correspondences);

		pl("true t");
		this.mock.getR(frameNum).times(this.mock.getIC(frameNum)).print(15, 5);

		// TRUE PURPOSE
		Matrix E = new Matrix(4, 4);
		E.set(0, 0, rt.getR().get(0, 0));
		E.set(0, 1, rt.getR().get(0, 1));
		E.set(0, 2, rt.getR().get(0, 2));
		E.set(1, 0, rt.getR().get(1, 0));
		E.set(1, 1, rt.getR().get(1, 1));
		E.set(1, 2, rt.getR().get(1, 2));
		E.set(2, 0, rt.getR().get(2, 0));
		E.set(2, 1, rt.getR().get(2, 1));
		E.set(2, 2, rt.getR().get(2, 2));
		E.set(0, 3, rt.getT().get(0, 0));
		E.set(1, 3, rt.getT().get(1, 0));
		E.set(2, 3, rt.getT().get(2, 0));

		E = ARUtils.sanitizeE(E);

		return this.updateTemporaryPoseFromCurrent(E);
	}

	// given a list of correspondences between the current keyframe and the
	// current frame, triangulate the 3D map points in the
	// current keyframe using the current keyframe pose and the current system
	// pose (which is passed in as "newPose")
	public ArrayList<Point3D> triangulateMapPoints(ArrayList<Correspondence2D2D> correspondences, Pose newPose) {
		ArrayList<Point3D> point3Ds = new ArrayList<Point3D>();
		for (int c = 0; c < correspondences.size(); c++) {
			Correspondence2D2D corr = correspondences.get(c);

			// isolate the keyframe entry to add a 3D point to
			MapPoint mapPoint = this.currentKeyFrame.getObsvToMapPoint()
					.get(corr.getU1().intValue() + "," + corr.getV1().intValue());

			Matrix E = newPose.getHomogeneousMatrix();
			Matrix pointMatrix = ARUtils.triangulate(E, this.currentKeyFrame.getPose().getHomogeneousMatrix(), corr);

			Point3D point3D = new Point3D(pointMatrix.get(0, 0), pointMatrix.get(1, 0), pointMatrix.get(2, 0));
			mapPoint.setPoint(point3D);
			point3Ds.add(point3D);
		}
		return point3Ds;
	}

	// given a list of correspondences between the current keyframe and the
	// current frame, triangulate the untracked 3D map points in the
	// current keyframe using the current keyframe pose and the current system
	// pose (which must be updated before calling this function)
	public void triangulateUntrackedMapPoints(ArrayList<Correspondence2D2D> correspondences) {
		for (int c = 0; c < correspondences.size(); c++) {
			Correspondence2D2D corr = correspondences.get(c);

			// get the map point corresponding to the point
			MapPoint mapPoint = this.currentKeyFrame.getObsvToMapPoint()
					.get(correspondences.get(c).getU1().intValue() + "," + correspondences.get(c).getV1().intValue());

			// ignore if this point has already been triangulated
			if (mapPoint.getPoint() != null) {
				continue;
			}

			// triangulate point, create 3D point, add to map point
			Matrix E = this.pose.getHomogeneousMatrix();
			Matrix pointMatrix = ARUtils.triangulate(E, this.currentKeyFrame.getPose().getHomogeneousMatrix(), corr);
			Point3D point3D = new Point3D(pointMatrix.get(0, 0), pointMatrix.get(1, 0), pointMatrix.get(2, 0));
			pl(point3D.getX() + ", " + point3D.getY() + ", " + point3D.getZ());
			mapPoint.setPoint(point3D);
		}
	}

	// given a list of correspondences, evaluate if there is sufficient
	// intra-correspondence difference to warrant triangulation
	public boolean sufficientMovement(ArrayList<Correspondence2D2D> correspondences) {

		// required average pixel difference to return a true value
		double REQ_DIST = 10;

		if (correspondences.size() == 0) {
			return false;
		}

		double sumDistance = 0.0;
		ArrayList<Double> distances = new ArrayList<Double>();
		for (int i = 0; i < correspondences.size(); i++) {
			Correspondence2D2D c = correspondences.get(i);
			double distance = Math.sqrt(Math.pow(c.getU1() - c.getU2(), 2) + Math.pow(c.getV1() - c.getV2(), 2));
			distances.add(distance);
			sumDistance += distance;
		}
		double avgDistance = sumDistance / correspondences.size();
		pl("avgDistance: " + avgDistance);

		return avgDistance > REQ_DIST;
	}

	// given 2 poses, their correspondences, and the corresponding 3D point
	// locations, perform a BA to correct large errors in triangulation and
	// scale
	public void cameraPairBundleAdjustment(Pose firstPose, Pose secondPose,
			ArrayList<Correspondence2D2D> correspondences, ArrayList<Point3D> point3Ds, int maxIterations) {

		ArrayList<Pose> cameras = new ArrayList<Pose>();
		cameras.add(firstPose);
		cameras.add(secondPose);

		ArrayList<ArrayList<Point2D>> obsv = new ArrayList<ArrayList<Point2D>>();
		for (int c = 0; c < correspondences.size(); c++) {
			ArrayList<Point2D> points = new ArrayList<Point2D>();
			Point2D pt1 = new Point2D(correspondences.get(c).getU1(), correspondences.get(c).getV1());
			Point2D pt2 = new Point2D(correspondences.get(c).getU2(), correspondences.get(c).getV2());
			points.add(pt1);
			points.add(pt2);
			obsv.add(points);
		}

		ARUtils.bundleAdjust(cameras, point3Ds, obsv, maxIterations);

	}

	public void PnPBAOptimize(Pose firstPose, Pose secondPose, ArrayList<Point2D> keypoints1,
			ArrayList<Point2D> keypoints2, ArrayList<Point3D> point3Ds) {
		ArrayList<ArrayList<Point2D>> obsv = new ArrayList<ArrayList<Point2D>>();
		for (int i = 0; i < point3Ds.size(); i++) {
			ArrayList<Point2D> points = new ArrayList<Point2D>();
			points.add(keypoints1.get(i));
			points.add(keypoints2.get(i));
			obsv.add(points);
		}

		ArrayList<Pose> cameras = new ArrayList<Pose>();
		cameras.add(firstPose);
		cameras.add(secondPose);

		ARUtils.bundleAdjust(cameras, point3Ds, obsv, 1);
	}

	public ArrayList<Point3D> get3DPoints(ArrayList<Correspondence2D2D> correspondences, KeyFrame keyframe) {
		ArrayList<Point3D> point3Ds = new ArrayList<Point3D>();
		for (int c = 0; c < correspondences.size(); c++) {
			point3Ds.add(keyframe.getObsvToMapPoint()
					.get(correspondences.get(c).getU1().intValue() + "," + correspondences.get(c).getV1().intValue())
					.getPoint());
		}
		return point3Ds;
	}

	public ArrayList<MapPoint> getMapPoints(ArrayList<Correspondence2D2D> correspondences, KeyFrame keyframe) {
		ArrayList<MapPoint> mapPoints = new ArrayList<MapPoint>();
		for (int c = 0; c < correspondences.size(); c++) {
			mapPoints.add(keyframe.getObsvToMapPoint()
					.get(correspondences.get(c).getU1().intValue() + "," + correspondences.get(c).getV1().intValue()));
		}
		return mapPoints;
	}

	public void PnPUpdate(ArrayList<Point3D> point3Ds, ArrayList<Correspondence2D2D> correspondences) {

		Mat rvec = new Mat();
		Mat tvec = new Mat();
		ARUtils.PnPRANSACPrune(correspondences, point3Ds, rvec, tvec);

		ArrayList<Point3D> tracked3DPoints = new ArrayList<Point3D>();
		ArrayList<Point2D> trackedKeypoints1 = new ArrayList<Point2D>();
		ArrayList<Point2D> trackedKeypoints2 = new ArrayList<Point2D>();
		for (int p = 0; p < point3Ds.size(); p++) {
			if (point3Ds.get(p) != null) {
				tracked3DPoints.add(point3Ds.get(p));
				Point2D point2D1 = new Point2D();
				point2D1.setX(correspondences.get(p).getU1());
				point2D1.setY(correspondences.get(p).getV1());
				trackedKeypoints1.add(point2D1);
				Point2D point2D2 = new Point2D();
				point2D2.setX(correspondences.get(p).getU2());
				point2D2.setY(correspondences.get(p).getV2());
				trackedKeypoints2.add(point2D2);
			}
		}

		pl("");
		pl("3D points used in PnP: ");
		for (int i = 0; i < tracked3DPoints.size(); i++) {
			pl(tracked3DPoints.get(i).getX() + ", " + tracked3DPoints.get(i).getY() + ", "
					+ tracked3DPoints.get(i).getZ());
		}
		pl("");

		Matrix E = ARUtils.OpenCVPnP(tracked3DPoints, trackedKeypoints2, rvec, tvec, true);
		Pose tempPose = this.setTemporaryPose(E);

		// debug
		pl("this.currentKeyframe.getPose(): ");
		this.currentKeyFrame.getPose().getHomogeneousMatrix().print(15, 5);
		pl("PnP pose: ");
		tempPose.getHomogeneousMatrix().print(15, 5);
		pl("trackedKeypoints1 size: " + trackedKeypoints1.size());
		pl("trackedKeypoints2 size: " + trackedKeypoints2.size());
		pl("tracked3DPoints size: " + tracked3DPoints.size());

		// bundle adjustment
		this.PnPBAOptimize(this.currentKeyFrame.getPose(), tempPose, trackedKeypoints1, trackedKeypoints2,
				tracked3DPoints);

		this.deepReplacePose(tempPose);
	}

	int count = 0;

	protected void mainloop() {
		boolean keepGoing = true;
		while (keepGoing) {

			if (this.frameNum >= this.mock.getMAX_FRAMES()) {
				keepGoing = false;
				continue;
			}

			// find ORB features (omit for mock)
			ArrayList<Point> pointsList = new ArrayList<Point>();
			Mat descriptors = this.mock.getKeypointsAndDescriptors(this.frameNum, pointsList);
			ArrayList<KeyPoint> keypointsList = new ArrayList<KeyPoint>();
			for (int i = 0; i < pointsList.size(); i++) {
				KeyPoint keypoint = new KeyPoint();
				keypoint.pt = pointsList.get(i);
				keypointsList.add(keypoint);
			}
			MatOfKeyPoint keypoints = new MatOfKeyPoint();
			keypoints.fromList(keypointsList);

			// if no keyframes exist, generate one
			if (this.map.getKeyframes().size() == 0) {
				this.currentKeyFrame = this.map.generateInitialKeyFrame(descriptors, keypoints, frameNum);
			}
			// b. otherwise,
			else {
				pl("===============================    FRAME " + this.frameNum
						+ "   ===================================");

				// match descriptors to those in currentKeyframe
				ArrayList<Correspondence2D2D> correspondences = new ArrayList<Correspondence2D2D>();
				ArrayList<Point> matchedKeyframePoints = new ArrayList<Point>();
				ArrayList<Point> matchedPoints = new ArrayList<Point>();
				matchDescriptors(descriptors, keypoints, this.currentKeyFrame, correspondences, matchedKeyframePoints,
						matchedPoints);
				pl("num correspondences: " + correspondences.size());

				// initialize the map (for mock purposes)
				if (!mapInitialized && this.sufficientMovement(correspondences)) {
					Pose newPose = this.structureFromMotionUpdate(matchedKeyframePoints, matchedPoints,
							correspondences);

					this.pose.getHomogeneousMatrix().print(15, 5);

					// triangulate points in map
					ArrayList<Point3D> point3Ds = this.triangulateMapPoints(correspondences, newPose);

					// single round BA to greatly correct poor results from sfm
					// and triangulation
					this.cameraPairBundleAdjustment(this.currentKeyFrame.getPose(), newPose, correspondences, point3Ds,
							1);

					this.deepReplacePose(newPose);

					mapInitialized = true;
				} else if (mapInitialized) {

					// get tracked 3D points
					ArrayList<Point3D> point3Ds = get3DPoints(correspondences, this.currentKeyFrame);
					int numTracked = 0;
					for (int i = 0; i < point3Ds.size(); i++) {
						numTracked += point3Ds.get(i) != null ? 1 : 0;
					}

					// General Pipeline Algorithm:

					// if >= 8 correspondences and > 16 tracked points
					// ---- track pose with PnP
					// ---- triangulate untracked points
					// else if >= 8 correspondences and 16 to 6 tracked points
					// ---- track pose with PnP and create new keyframe
					// else if >= 8 correspondences and < 6 tracked points
					// ---- sfm track?
					// else (< 8 correspondences)
					// ---- place recognition module
					pl("numCorrespondences: " + correspondences.size());
					pl("numTracked: " + numTracked);

					if (correspondences.size() >= 8 && numTracked > 16) {
						// PnP
						this.PnPUpdate(point3Ds, correspondences);

						// Triangulate untracked map points
						if (this.sufficientMovement(correspondences)) {
							this.triangulateUntrackedMapPoints(correspondences);
						}

					} else if (correspondences.size() >= 8 && numTracked <= 16 && numTracked >= 6) {
						// PnP
						this.PnPUpdate(point3Ds, correspondences);

						// Triangulate untracked map points
						if (this.sufficientMovement(correspondences)) {
							this.triangulateUntrackedMapPoints(correspondences);
							// Create new keyframe
							this.currentKeyFrame = map.registerNewKeyframe(descriptors, keypoints, frameNum, this.pose,
									correspondences, this.getMapPoints(correspondences, this.currentKeyFrame));
						}

					}
				}
			}

			pl("num keyframes: " + this.map.getKeyframes().size());

			synchronized (this.outputPoseBuffer) {
				this.outputPoseBuffer.pushPose(pose);
			}

			// for demo, just push the unaltered frame along to the output
			// buffer
			synchronized (this.outputFrameBuffer) {
				int width = 480;
				int height = 270;

				Frame currentFrame = ARUtils.artificialKeypointFrame(pointsList, width, height);
				this.outputFrameBuffer.pushFrame(currentFrame);
			}

			this.frameNum++;

			try {
				if (frameNum > 180) {
					// Thread.sleep(1000);
				}

			} catch (Exception e) {

			}

			// record framerate
			long newTime = System.nanoTime();
			double frameTime = (newTime - lastTime) / 1000000000.0;
			double framerate = 1 / frameTime;
			lastTime = newTime;
			pl("framerate:\t\t" + (int) framerate);
		}
	}

	public void getStats(Mat desc) {
		for (int i = 0; i < desc.cols(); i++) {
			ArrayList<Double> column = new ArrayList<Double>();
			for (int j = 0; j < desc.rows(); j++) {
				column.add(desc.get(j, i)[0]);
			}
			System.out.println("====  " + i + "  ===============================");
			System.out.println("Min    : " + ARUtils.min(column));
			System.out.println("Max    : " + ARUtils.max(column));
			System.out.println("Average: " + ARUtils.mean(column));
			System.out.println("Std Dev: " + ARUtils.stdDev(column));
		}
	}

	public static void p(Object s) {
		System.out.print(s);
	}

	public static void pl(Object s) {
		System.out.println(s);
	}

}