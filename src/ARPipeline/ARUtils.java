package ARPipeline;

import java.util.ArrayList;
import java.util.List;

import org.ddogleg.optimization.lm.ConfigLevenbergMarquardt;
import org.ejml.data.DMatrixRMaj;
import org.lwjgl.util.vector.Matrix4f;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.features2d.ORB;

import Jama.Matrix;
import Jama.SingularValueDecomposition;
import boofcv.abst.geo.bundle.BundleAdjustment;
import boofcv.abst.geo.bundle.ScaleSceneStructure;
import boofcv.abst.geo.bundle.SceneObservations;
import boofcv.abst.geo.bundle.SceneStructureMetric;
import boofcv.alg.geo.bundle.cameras.BundlePinhole;
import boofcv.factory.geo.ConfigBundleAdjustment;
import boofcv.factory.geo.FactoryMultiView;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;

public class ARUtils {

	public static ArrayList<Mat> decomposeDescriptorMat(Mat descriptors) {
		ArrayList<Mat> descList = new ArrayList<Mat>();
		for (int i = 0; i < descriptors.rows(); i++) {
			descList.add(descriptors.row(i));
		}
		return descList;
	}

	public static Matrix getPlane(KeyFrame keyframe) {
		ArrayList<Point3D> points = new ArrayList<Point3D>();
		pl("points:");
		for (int i = 0; i < keyframe.getMapPoints().size(); i++) {
			if (keyframe.getMapPoints().get(i).getPoint() != null) {
				Point3D p = keyframe.getMapPoints().get(i).getPoint();
				points.add(p);
				pl(p.getX() + ", " + p.getY() + ", " + p.getZ());
			}
		}
		return getPlane(points);

	}

	// given non-null 3D points, calculate the a, b, c, and d of a best fit
	// plane between them
	public static Matrix getPlane(ArrayList<Point3D> points) {
		Matrix pointMatrix = new Matrix(points.size(), 4);
		for (int i = 0; i < points.size(); i++) {
			pointMatrix.set(i, 0, points.get(i).getX());
			pointMatrix.set(i, 1, points.get(i).getY());
			pointMatrix.set(i, 2, points.get(i).getZ());
			pointMatrix.set(i, 3, 1);
		}

		SingularValueDecomposition svd = pointMatrix.svd();

		return svd.getV().getMatrix(0, 3, 3, 3);
	}

	public static Matrix fundamentalMatrix(Pose pose, Matrix K) {
		Matrix R = pose.getRotationMatrix().getMatrix(0, 2, 0, 2);
		Matrix tx = new Matrix(3, 3);
		tx.set(0, 1, -pose.getTz());
		tx.set(0, 2, pose.getTy());
		tx.set(1, 0, pose.getTz());
		tx.set(1, 2, -pose.getTx());
		tx.set(2, 0, -pose.getTy());
		tx.set(2, 1, pose.getTx());

		Matrix essential = tx.times(R);

		Matrix KInv = K.getMatrix(0, 2, 0, 2).inverse();

		Matrix fundamental = KInv.transpose().times(essential).times(KInv);

		return fundamental;
	}

	public static Integer matchDescriptor(Mat baseDescriptor, List<KeyPoint> keypoints, List<Mat> descriptors) {
		return matchDescriptor(baseDescriptor, keypoints, descriptors, 0.65, 25);
	}

	public static Integer matchDescriptor(Mat baseDescriptor, List<KeyPoint> keypoints, List<Mat> descriptors,
			double lowe, double thresh) {
		if (keypoints.size() == 0)
			return null;

		double LOWE_RATIO = lowe;
		double DIST_THRESH = thresh;

		double[] hammingDistances = new double[descriptors.size()];

		// get hamming distances for each descriptor
		for (int i = 0; i < descriptors.size(); i++) {
			hammingDistances[i] = Core.norm(descriptors.get(i), baseDescriptor, Core.NORM_HAMMING);
		}

		// get best and second best distances
		double bestDistance = 256;
		int bestIndex = -1;
		double secondDistance = 257;

		for (int i = 0; i < hammingDistances.length; i++) {
			if (hammingDistances[i] < bestDistance) {
				secondDistance = bestDistance;
				bestDistance = hammingDistances[i];
				bestIndex = i;
			} else if (hammingDistances[i] < secondDistance) {
				secondDistance = hammingDistances[i];
			}
		}

		// evaluate Lowe ratio and absolute distance
		if (bestDistance < LOWE_RATIO * secondDistance && bestDistance < DIST_THRESH) {
			return bestIndex;
		}

		// no sufficient match
		return null;
	}

	public static Integer matchDescriptor(Mat baseDescriptor, MatOfKeyPoint keypoints, Mat descriptors) {
		return matchDescriptor(baseDescriptor, keypoints, descriptors, 0.7, 25);
	}

	public static Integer matchDescriptor(Mat baseDescriptor, MatOfKeyPoint keypoints, Mat descriptors, double lowe,
			double thresh) {
		if (keypoints.rows() == 0)
			return null;

		double LOWE_RATIO = lowe;
		double DIST_THRESH = thresh;

		double[] hammingDistances = new double[descriptors.rows()];

		// get hamming distances for each descriptor
		for (int i = 0; i < descriptors.rows(); i++) {
			hammingDistances[i] = Core.norm(descriptors.row(i), baseDescriptor, Core.NORM_HAMMING);
		}

		// get best and second best distances
		double bestDistance = 256;
		int bestIndex = -1;
		double secondDistance = 257;

		for (int i = 0; i < hammingDistances.length; i++) {
			if (hammingDistances[i] < bestDistance) {
				secondDistance = bestDistance;
				bestDistance = hammingDistances[i];
				bestIndex = i;
			} else if (hammingDistances[i] < secondDistance) {
				secondDistance = hammingDistances[i];
			}
		}

		// evaluate Lowe ratio and absolute distance
		if (bestDistance < LOWE_RATIO * secondDistance && bestDistance < DIST_THRESH) {
			return bestIndex;
		}

		// no sufficient match
		return null;
	}

	public static void getFeaturesInWindow(Frame frame, Frame outputFrame, int x, int y, int windowSize,
			java.util.HashMap<String, ArrayList<KeyPoint>> keypointMap,
			java.util.HashMap<String, ArrayList<Mat>> descriptorMap, List<KeyPoint> windowKeypoints,
			List<Mat> windowDescriptors) {

		// ensure lists are empty
		windowKeypoints.clear();
		windowDescriptors.clear();

		// create window
		int startX = x - windowSize < 0 ? 0 : x - windowSize;
		int endX = x + windowSize > frame.getWidth() ? frame.getWidth() : x + windowSize;
		int startY = y - windowSize < 0 ? 0 : y - windowSize;
		int endY = y + windowSize > frame.getHeight() ? frame.getHeight() : y + windowSize;

		// handle cases where search area is completely off-screen
		if (startX >= frame.getWidth())
			return;
		if (endX <= 0)
			return;
		if (startY >= frame.getHeight())
			return;
		if (endY <= 0)
			return;

		// collect features in window
		for (int row = startY; row < endY; row++) {
			for (int col = startX; col < endX; col++) {
				if (keypointMap.get(col + "," + row) == null)
					continue;
				ArrayList<KeyPoint> keypointsInMap = keypointMap.get(col + "," + row);
				ArrayList<Mat> descriptorsInMap = descriptorMap.get(col + "," + row);
				windowKeypoints.addAll(keypointsInMap);
				windowDescriptors.addAll(descriptorsInMap);
			}
		}

		byte[] purple = { (byte) 255, 0, (byte) 255 };
		byte[] green = { 0, (byte) 255, 0 };
		byte[] red = { (byte) 255, 0, 0 };
		byte[] color = windowKeypoints.size() == 0 ? red : purple;
		// boxHighlight(outputFrame, x, y, color, endX - startX, endY - startY);
		// boxHighlight(outputFrame, oKeypoints, patchSize);

	}

	public static void getFeaturesInWindow(Frame frame, Frame outputFrame, Mat image, int x, int y, int windowSize,
			MatOfKeyPoint oKeypoints, Mat oDescriptors) {
		getFeaturesInWindow(frame, outputFrame, image, x, y, windowSize, oKeypoints, oDescriptors, 5);
	}

	// given a small search window and coordinates for the center of the window,
	// find a number of features in the image within only the window
	public static void getFeaturesInWindow(Frame frame, Frame outputFrame, Mat image, int x, int y, int windowSize,
			MatOfKeyPoint oKeypoints, Mat oDescriptors, int numFeatures) {

		// ORB parameters
		int patchSize = 12;
		ORB orb = ORB.create(numFeatures);
		orb.setScoreType(ORB.FAST_SCORE);
		orb.setPatchSize(patchSize);
		orb.setNLevels(1);
		orb.setScaleFactor(1.5);
		orb.setEdgeThreshold(0);

		// create window
		int startX = x - windowSize < 0 ? 0 : x - windowSize;
		int endX = x + windowSize > frame.getWidth() ? frame.getWidth() : x + windowSize;
		int startY = y - windowSize < 0 ? 0 : y - windowSize;
		int endY = y + windowSize > frame.getHeight() ? frame.getHeight() : y + windowSize;

		// handle cases where search area is completely off-screen
		if (startX >= frame.getWidth())
			return;
		if (endX <= 0)
			return;
		if (startY >= frame.getHeight())
			return;
		if (endY <= 0)
			return;

		Mat subImage = image.submat(startY, endY, startX, endX);
		orb.detect(subImage, oKeypoints);
		orb.compute(subImage, oKeypoints, oDescriptors);

		// offset the keypoints
		for (int i = 0; i < oKeypoints.rows(); i++) {
			for (int j = 0; j < oKeypoints.cols(); j++) {
				oKeypoints.put(i, j, oKeypoints.get(i, j)[0] + startX, oKeypoints.get(i, j)[1] + startY,
						oKeypoints.get(i, j)[2], oKeypoints.get(i, j)[3], oKeypoints.get(i, j)[4],
						oKeypoints.get(i, j)[5], oKeypoints.get(i, j)[6]);
			}
		}

		byte[] purple = { (byte) 255, 0, (byte) 255 };
		byte[] green = { 0, (byte) 255, 0 };
		byte[] red = { (byte) 255, 0, 0 };
		byte[] color = oKeypoints.rows() == 0 ? red : green;
		// boxHighlight(outputFrame, x, y, color, endX - startX, endY - startY);
		// boxHighlight(outputFrame, oKeypoints, patchSize);

	}

	public static void fullFrameFeatureDetect(Frame currentFrame, Frame outputFrame, MatOfKeyPoint keypoints,
			Mat descriptors, int edgeThreshold, boolean paintFindings) {

		Mat image = ARUtils.frameToMat(currentFrame);

		// ORB parameters
		int patchSize = 12;
		ORB orb = ORB.create(1000);
		orb.setScoreType(ORB.FAST_SCORE);
		orb.setPatchSize(patchSize);
		orb.setNLevels(1);
		orb.setScaleFactor(1.5);
		orb.setEdgeThreshold(edgeThreshold); // 5

		int GRID_DIM_X = 1;
		int GRID_DIM_Y = 1;
		int CELL_WIDTH = (int) Math.floor(currentFrame.getWidth() / GRID_DIM_X);
		int CELL_HEIGHT = (int) Math.floor(currentFrame.getHeight() / GRID_DIM_Y);

		ArrayList<ArrayList<MatOfKeyPoint>> allKeypoints = new ArrayList<ArrayList<MatOfKeyPoint>>();

		// extract ORB features in each cell
		for (int i = 0; i < GRID_DIM_X; i++) {

			allKeypoints.add(new ArrayList<MatOfKeyPoint>());

			for (int j = 0; j < GRID_DIM_Y; j++) {

				int startX = i * CELL_WIDTH;
				int endX = (i + 1) * CELL_WIDTH;
				int startY = j * CELL_HEIGHT;
				int endY = (j + 1) * CELL_HEIGHT;

				// pl("startX: " + startX);
				// pl("endX: " + endX);
				// pl("startY: " + startY);
				// pl("endY: " + endY);
				Mat subImage = image.submat(startY, endY, startX, endX);

				// find ORB features
				MatOfKeyPoint tempKeypoints = new MatOfKeyPoint();
				long start = System.currentTimeMillis();
				orb.detect(subImage, tempKeypoints);
				long end = System.currentTimeMillis();
				// pl("full feature detect cell: " + (end - start) + " ms");

				// offset the keypoints
				for (int r = 0; r < tempKeypoints.rows(); r++) {
					for (int c = 0; c < tempKeypoints.cols(); c++) {
						tempKeypoints.put(r, c, tempKeypoints.get(r, c)[0] + startX,
								tempKeypoints.get(r, c)[1] + startY, tempKeypoints.get(r, c)[2],
								tempKeypoints.get(r, c)[3], tempKeypoints.get(r, c)[4], tempKeypoints.get(r, c)[5],
								tempKeypoints.get(r, c)[6]);
					}
				}

				allKeypoints.get(i).add(tempKeypoints);
			}
		}

		// merge keypoints into single list
		List<KeyPoint> keypointList = new ArrayList<KeyPoint>();
		for (int i = 0; i < allKeypoints.size(); i++) {
			for (int j = 0; j < allKeypoints.get(i).size(); j++) {
				List<KeyPoint> tempKeypointList = allKeypoints.get(i).get(j).toList();
				for (int k = 0; k < tempKeypointList.size(); k++) {
					keypointList.add(tempKeypointList.get(k));
				}
			}
		}

		keypoints.fromList(keypointList);
		nonMaximumSuppression(keypoints, patchSize);
		orb.compute(image, keypoints, descriptors);

		if (paintFindings)
			ARUtils.boxHighlight(outputFrame, keypoints, patchSize);

	}

	public static void extractFeatures(Frame currentFrame, MatOfKeyPoint keypoints, Mat descriptors) {

		Mat image = ARUtils.frameToMat(currentFrame);

		// ORB parameters
		int patchSize = 12;
		ORB orb = ORB.create(200);
		orb.setScoreType(ORB.FAST_SCORE);
		orb.setPatchSize(patchSize);
		orb.setNLevels(1);
		orb.setScaleFactor(1.5);

		// cell parameters
		int GRID_DIM_X = 1;
		int GRID_DIM_Y = 1;
		int CELL_WIDTH = (int) Math.floor(currentFrame.getWidth() / GRID_DIM_X);
		int CELL_HEIGHT = (int) Math.floor(currentFrame.getHeight() / GRID_DIM_Y);

		ArrayList<ArrayList<MatOfKeyPoint>> allKeypoints = new ArrayList<ArrayList<MatOfKeyPoint>>();

		// extract ORB features in each cell
		for (int i = 0; i < GRID_DIM_X; i++) {

			allKeypoints.add(new ArrayList<MatOfKeyPoint>());

			for (int j = 0; j < GRID_DIM_Y; j++) {
				Mat mask = generateMask(i * CELL_WIDTH, (i + 1) * CELL_WIDTH, j * CELL_HEIGHT, (j + 1) * CELL_HEIGHT,
						currentFrame.getWidth(), currentFrame.getHeight());

				// find ORB features
				MatOfKeyPoint tempKeypoints = new MatOfKeyPoint();
				long start = System.currentTimeMillis();
				orb.detect(image, tempKeypoints, mask);
				// orb.detect(image, tempKeypoints);
				long end = System.currentTimeMillis();
				pl("full feature detect: " + (end - start) + " ms");

				allKeypoints.get(i).add(tempKeypoints);
			}
		}

		// merge keypoints into single list
		List<KeyPoint> keypointList = new ArrayList<KeyPoint>();
		for (int i = 0; i < allKeypoints.size(); i++) {
			for (int j = 0; j < allKeypoints.get(i).size(); j++) {
				List<KeyPoint> tempKeypointList = allKeypoints.get(i).get(j).toList();
				for (int k = 0; k < tempKeypointList.size(); k++) {
					keypointList.add(tempKeypointList.get(k));
				}
			}
		}

		keypoints.fromList(keypointList);

		// pruneScaleDependentFeatures(keypoints, 2, 5);
		// nonMaximumSuppression(keypoints, patchSize);

		ARUtils.pruneKeypoints(keypoints, 50);
		orb.compute(image, keypoints, descriptors);

		ARUtils.boxHighlight(currentFrame, keypoints, patchSize);
		// ARUtils.boxHighlight(currentFrame, keypoints, descriptors,
		// patchSize);
	}

	//
	public static void pruneScaleDependentFeatures(MatOfKeyPoint keypoints, int NUM_SCALES_REQ, int RANGE) {
		List<KeyPoint> keypointList = keypoints.toList();

		// many features will show up at several levels. When finding these
		// features, group them up with their entries at all levels they show
		// up.
		ArrayList<ArrayList<KeyPoint>> goodKeypointGroupings = new ArrayList<ArrayList<KeyPoint>>();

		// to make it faster to find close keypoints, load them into hashmap
		java.util.HashMap<String, ArrayList<KeyPoint>> keypointMap = new java.util.HashMap<String, ArrayList<KeyPoint>>();
		for (KeyPoint keypoint : keypointList) {
			ArrayList<KeyPoint> keypointsHere = keypointMap.get((int) keypoint.pt.x + "," + (int) keypoint.pt.y);
			if (keypointsHere == null) {
				keypointsHere = new ArrayList<KeyPoint>();
				keypointMap.put((int) keypoint.pt.x + "," + (int) keypoint.pt.y, keypointsHere);
			}
			keypointsHere.add(keypoint);
		}

		// go through every keypoint and find any other keypoints that are close
		// to it and group them up
		for (KeyPoint keypoint : keypointList) {
			ArrayList<KeyPoint> group = new ArrayList<KeyPoint>();
			for (int i = (int) keypoint.pt.x - RANGE; i <= (int) keypoint.pt.x + RANGE; i++) {
				for (int j = (int) keypoint.pt.y - RANGE; j <= (int) keypoint.pt.y + RANGE; j++) {
					ArrayList<KeyPoint> entries = keypointMap.get(i + "," + j);
					if (entries == null) {
						continue;
					}
					for (KeyPoint entry : entries) {
						group.add(entry);
					}
				}
			}
			if (group.size() >= NUM_SCALES_REQ) {
				goodKeypointGroupings.add(group);
			}
		}

		// pick and retain the best keypoint from each group
		ArrayList<KeyPoint> prunedKeypoints = new ArrayList<KeyPoint>();
		for (int i = 0; i < goodKeypointGroupings.size(); i++) {
			KeyPoint bestKeypoint = goodKeypointGroupings.get(i).get(0);
			double bestResponse = goodKeypointGroupings.get(i).get(0).response;
			for (int j = 1; j < goodKeypointGroupings.get(i).size(); j++) {
				if (goodKeypointGroupings.get(i).get(j).response < bestResponse) {
					bestKeypoint = goodKeypointGroupings.get(i).get(j);
					bestResponse = bestKeypoint.response;
				}
			}
			prunedKeypoints.add(bestKeypoint);
		}

		pl("keypointList.size(): " + keypointList.size());
		pl("prunedKeypoints.size(): " + prunedKeypoints.size());
		keypoints.fromList(prunedKeypoints);
	}

	public static Mat generateMask(int startX, int endX, int startY, int endY, int frameWidth, int frameHeight) {

		Mat mask = Mat.zeros(frameHeight, frameWidth, CvType.CV_8U);
		for (int i = startX; i < endX; i++) {
			for (int j = startY; j < endY; j++) {
				mask.put(j, i, 255);
			}
		}

		return mask;
	}

	public static Matrix sanitizeE(Matrix E) {

		// extract rotation
		Matrix R = E.copy().getMatrix(0, 2, 0, 2);

		// invert rotation
		Matrix RInv = R.inverse();

		// get C
		Matrix C = RInv.times(E.getMatrix(0, 2, 3, 3)).times(-1);

		// convert rotation to quaternion and normalize
		DMatrixRMaj rotation = new DMatrixRMaj(3, 3);
		rotation.add(0, 0, R.get(0, 0));
		rotation.add(0, 1, R.get(0, 1));
		rotation.add(0, 2, R.get(0, 2));
		rotation.add(1, 0, R.get(1, 0));
		rotation.add(1, 1, R.get(1, 1));
		rotation.add(1, 2, R.get(1, 2));
		rotation.add(2, 0, R.get(2, 0));
		rotation.add(2, 1, R.get(2, 1));
		rotation.add(2, 2, R.get(2, 2));

		Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(rotation, null);
		q.normalize();

		// convert quaternion back to rotation
		rotation = ConvertRotation3D_F64.quaternionToMatrix(q, null);
		Matrix newR = Matrix.identity(4, 4);
		newR.set(0, 0, rotation.get(0, 0));
		newR.set(0, 1, rotation.get(0, 1));
		newR.set(0, 2, rotation.get(0, 2));
		newR.set(1, 0, rotation.get(1, 0));
		newR.set(1, 1, rotation.get(1, 1));
		newR.set(1, 2, rotation.get(1, 2));
		newR.set(2, 0, rotation.get(2, 0));
		newR.set(2, 1, rotation.get(2, 1));
		newR.set(2, 2, rotation.get(2, 2));

		Matrix IC = Matrix.identity(4, 4);
		IC.set(0, 3, -C.get(0, 0));
		IC.set(1, 3, -C.get(1, 0));
		IC.set(2, 3, -C.get(2, 0));

		// multiply homogeneous rotation with IC
		Matrix newE = newR.times(IC);
		return newE;

	}

	// observations is a list of size n (points) with inner lists of size m
	// (cameras). If 3 cameras and 10 points, observations is a list of size
	// 10,
	// with the sublists being size 3
	public static void bundleAdjust(ArrayList<Pose> cameras, ArrayList<Point3D> point3Ds,
			ArrayList<ArrayList<Point2D>> obsv, int maxIterations) {

		// boofCV
		SceneStructureMetric scene = new SceneStructureMetric(false);
		scene.initialize(cameras.size(), cameras.size(), point3Ds.size());
		SceneObservations observations = new SceneObservations();
		observations.initialize(cameras.size());

		// load camera poses into scene
		BundlePinhole camera = new BundlePinhole();
		camera.fx = CameraIntrinsics.fx;
		camera.fy = CameraIntrinsics.fy;
		camera.cx = CameraIntrinsics.cx;
		camera.cy = CameraIntrinsics.cy;
		camera.skew = CameraIntrinsics.s;
		for (int i = 0; i < cameras.size(); i++) {
			Se3_F64 worldToCameraGL = new Se3_F64();
			ConvertRotation3D_F64.quaternionToMatrix(cameras.get(i).getQw(), cameras.get(i).getQx(),
					cameras.get(i).getQy(), cameras.get(i).getQz(), worldToCameraGL.R);
			worldToCameraGL.T.x = cameras.get(i).getTx();
			worldToCameraGL.T.y = cameras.get(i).getTy();
			worldToCameraGL.T.z = cameras.get(i).getTz();
			scene.setCamera(i, true, camera);
			scene.setView(i, cameras.get(i).isFixed(), worldToCameraGL);
			scene.connectViewToCamera(i, i);
		}

		// load projected observations into observations variable
		for (int pointID = 0; pointID < obsv.size(); pointID++) {
			for (int cameraID = 0; cameraID < obsv.get(pointID).size(); cameraID++) {
				float pixelX = (float) obsv.get(pointID).get(cameraID).getX();
				float pixelY = (float) obsv.get(pointID).get(cameraID).getY();
				observations.getView(cameraID).add(pointID, pixelX, pixelY);
			}
		}

		// load 3D points into scene
		for (int i = 0; i < point3Ds.size(); i++) {
			float x = (float) point3Ds.get(i).getX();
			float y = (float) point3Ds.get(i).getY();
			float z = (float) point3Ds.get(i).getZ();

			scene.setPoint(i, x, y, z);
		}

		ConfigLevenbergMarquardt configLM = new ConfigLevenbergMarquardt();
		configLM.dampeningInitial = 1e-3;
		configLM.hessianScaling = true;

		ConfigBundleAdjustment configSBA = new ConfigBundleAdjustment();
		configSBA.configOptimizer = configLM;
		BundleAdjustment<SceneStructureMetric> bundleAdjustment = FactoryMultiView.bundleSparseMetric(configSBA);

		// debug
		bundleAdjustment.setVerbose(System.out, 0);

		// Specifies convergence criteria
		bundleAdjustment.configure(1e-12, 1e-12, maxIterations);

		// Scaling each variable type so that it takes on a similar numerical
		// value. This aids in optimization
		// Not important for this problem but is for others
		ScaleSceneStructure bundleScale = new ScaleSceneStructure();
		bundleScale.applyScale(scene, observations);
		bundleAdjustment.setParameters(scene, observations);

		// Runs the solver. This will take a few minutes. 7 iterations takes
		// about 3 minutes on my computer
		long startTime = System.currentTimeMillis();
		double errorBefore = bundleAdjustment.getFitScore();
		pl("error before: " + errorBefore);
		if (!bundleAdjustment.optimize(scene)) {
			// throw new RuntimeException("Bundle adjustment failed?!?");
			pl("\n\n\n\n***************************  ERROR  ****************************");
			pl("NOTE: Bundle Adjustment failed!");
			pl("fit score: " + bundleAdjustment.getFitScore());
			bundleScale.undoScale(scene, observations);

			pl("original 3D points ==> scene 3D points: ");
			for (int i = 0; i < point3Ds.size(); i++) {
				p(point3Ds.get(i).getX() + ",\t");
				p(point3Ds.get(i).getY() + ",\t");
				p(point3Ds.get(i).getZ() + "\t ====> \t\t");
				p(scene.getPoints().get(i).getX() + ",\t");
				p(scene.getPoints().get(i).getY() + ",\t");
				p(scene.getPoints().get(i).getZ() + "\n");
			}

			pl("original cameras: ");
			for (int i = 0; i < cameras.size(); i++) {
				pl("");
				cameras.get(i).getHomogeneousMatrix().print(15, 7);
				pl("");
			}

			pl("****************************************************************\n\n\n\n");

			return;
		}

		// Print out how much it improved the model
		System.out.println();
		System.out.printf("Error reduced by %.1f%%\n", (100.0 * (errorBefore / bundleAdjustment.getFitScore() - 1.0)));
		System.out.println((System.currentTimeMillis() - startTime) / 1000.0);

		// Return parameters to their original scaling. Can probably skip this
		// step.
		bundleScale.undoScale(scene, observations);

		// load points from scene back into input
		for (int i = 0; i < scene.getPoints().size(); i++) {
			point3Ds.get(i).setX(scene.getPoints().get(i).getX());
			point3Ds.get(i).setY(scene.getPoints().get(i).getY());
			point3Ds.get(i).setZ(scene.getPoints().get(i).getZ());
		}

		// load poses from scene back into input
		for (int viewID = 0; viewID < cameras.size(); viewID++) {
			Se3_F64 worldToView = scene.getViews().get(viewID).worldToView;
			Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(worldToView.getR(), null);
			q.normalize();
			Vector3D_F64 t = worldToView.getTranslation();
			cameras.get(viewID).setQw(q.w);
			cameras.get(viewID).setQx(q.x);
			cameras.get(viewID).setQy(q.y);
			cameras.get(viewID).setQz(q.z);
			cameras.get(viewID).setT(t.x, t.y, t.z);
		}

	}

	// return pseudo inverse of Matrix X using SVD
	public static Matrix pseudoInverse(Matrix X) {
		SingularValueDecomposition svd = X.svd();
		Matrix sPrime = svd.getS().copy();
		for (int i = 0; i < sPrime.getRowDimension(); i++) {
			sPrime.set(i, i, sPrime.get(i, i) == 0 ? 0 : (1 / sPrime.get(i, i)));
		}
		return svd.getV().times(sPrime).times(svd.getU().transpose());
	}

	// quaternion rotation, assuming column vector of format [qw, qx,
	// qy, qz].transpose()
	public static Matrix quatRot(Matrix q1, Matrix q2) {
		Matrix rot = quatMult(q1, quatMult(q2, quatInv(q1)));
		rot = rot.times(1 / rot.normF());
		return rot;
	}

	// quaternion inverse/reciprocal, assuming column vector of format [qw, qx,
	// qy, qz].transpose()
	public static Matrix quatInv(Matrix q) {
		Matrix inv = q.copy().times(-1);
		inv.set(0, 0, -inv.get(0, 0));
		inv = inv.times(1 / Math.pow(inv.normF(), 2));
		return inv;
	}

	// quaternion multiplication, assuming column vector of format [qw, qx, qy,
	// qz].transpose() (q*r)
	public static Matrix quatMult(Matrix q, Matrix r) {

		Matrix t = new Matrix(4, 1);

		double q0 = q.get(0, 0);
		double q1 = q.get(1, 0);
		double q2 = q.get(2, 0);
		double q3 = q.get(3, 0);
		double r0 = r.get(0, 0);
		double r1 = r.get(1, 0);
		double r2 = r.get(2, 0);
		double r3 = r.get(3, 0);

		t.set(0, 0, r0 * q0 - r1 * q1 - r2 * q2 - r3 * q3);
		t.set(1, 0, r0 * q1 + r1 * q0 - r2 * q3 + r3 * q2);
		t.set(2, 0, r0 * q2 + r1 * q3 + r2 * q0 - r3 * q1);
		t.set(3, 0, r0 * q3 - r1 * q2 + r2 * q1 + r3 * q0);

		t = t.times(1 / t.normF());

		return t;
	}

	public static Matrix getOrthogonalized(Matrix R, int limit) {

		Matrix I = Matrix.identity(3, 3);
		Matrix sum = Matrix.identity(3, 3);
		Matrix temp;
		Matrix X;
		Matrix XPower = Matrix.identity(3, 3);

		double coef[] = { 1, -1 / 2., 3 / 8., -5 / 16., 35 / 128., -63 / 256., 231 / 1024., -429 / 2048., 6435 / 32768.,
				-12155 / 65536. };

		limit = Math.max(limit, 10);

		temp = R.transpose().getMatrix(0, 2, 0, 2).times(R.getMatrix(0, 2, 0, 2));
		X = temp.minus(I);

		for (int power = 1; power < limit; power++) {
			XPower = XPower.times(X);
			temp = XPower.times(coef[power]);
			sum = sum.plus(temp);
		}

		return R.getMatrix(0, 2, 0, 2).times(sum);

	}

	public static Matrix quaternionToRotationMatrix(Matrix Q) {
		double q1 = Q.get(0, 0);
		double q2 = Q.get(1, 0);
		double q3 = Q.get(2, 0);
		double q4 = Q.get(3, 0);
		Matrix R = Matrix.identity(4, 4);

		// R.set(0, 0, q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4);
		// R.set(0, 1, 2 * (q2 * q3 - q1 * q4));
		// R.set(0, 2, 2 * (q2 * q4 + q1 * q3));
		// R.set(1, 0, 2 * (q2 * q3 + q1 * q4));
		// R.set(1, 1, q1 * q1 - q2 * q2 + q3 * q3 - q4 * q4);
		// R.set(1, 2, 2 * (q2 * q4 - q1 * q2));
		// R.set(2, 0, 2 * (q2 * q4 - q1 * q3));
		// R.set(2, 1, 2 * (q3 * q4 + q1 * q2));
		// R.set(2, 2, q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4);

		double qw = Q.get(0, 0);
		double qx = Q.get(1, 0);
		double qy = Q.get(2, 0);
		double qz = Q.get(3, 0);

		R.set(0, 0, 1 - 2 * qy * qy - 2 * qz * qz);
		R.set(0, 1, 2 * qx * qy - 2 * qz * qw);
		R.set(0, 2, 2 * qx * qz + 2 * qy * qw);
		R.set(1, 0, 2 * qx * qy + 2 * qz * qw);
		R.set(1, 1, 1 - 2 * qx * qx - 2 * qz * qz);
		R.set(1, 2, 2 * qy * qz - 2 * qx * qw);
		R.set(2, 0, 2 * qx * qz - 2 * qy * qw);
		R.set(2, 1, 2 * qy * qz + 2 * qx * qw);
		R.set(2, 2, 1 - 2 * qx * qx - 2 * qy * qy);

		return R;
	}

	public static Matrix getQuaternion(Matrix R) {
		// index starting with 1
		double r11 = R.get(0, 0);
		double r12 = R.get(0, 1);
		double r13 = R.get(0, 2);
		double r21 = R.get(1, 0);
		double r22 = R.get(1, 1);
		double r23 = R.get(1, 2);
		double r31 = R.get(2, 0);
		double r32 = R.get(2, 1);
		double r33 = R.get(2, 2);

		// using Sarabandi-Thomas method
		double n = -1;
		double q1 = 0;
		double q2 = 0;
		double q3 = 0;
		double q4 = 0;

		// q1
		if (r11 + r22 + r33 > n) {
			q1 = 0.5 * Math.sqrt(1 + r11 + r22 + r33);
		} else {
			q1 = 0.5 * Math.sqrt(((r32 - r23) * (r32 - r23) + (r13 - r31) * (r13 - r31) + (r21 - r12) * (r21 - r12))
					/ (3 - r11 - r22 - r33));
		}

		// q2
		if (r11 - r22 - r33 > n) {
			q2 = 0.5 * Math.sqrt(1 + r11 - r22 - r33);
		} else {
			q2 = 0.5 * Math.sqrt(((r32 - r23) * (r32 - r23) + (r12 + r21) * (r12 + r21) + (r31 + r13) * (r31 + r13))
					/ (3 - r11 + r22 + r33));
		}

		// q3
		if (-r11 + r22 - r33 > n) {
			q3 = 0.5 * Math.sqrt(1 - r11 + r22 - r33);
		} else {
			q3 = 0.5 * Math.sqrt(((r13 - r31) * (r13 - r31) + (r12 + r21) * (r12 + r21) + (r23 + r32) * (r23 + r32))
					/ (3 + r11 - r22 + r33));
		}

		// q4
		if (-r11 - r22 + r33 > n) {
			q4 = 0.5 * Math.sqrt(1 - r11 - r22 + r33);
		} else {
			q4 = 0.5 * Math.sqrt(((r21 - r12) * (r21 - r12) + (r31 + r13) * (r31 + r13) + (r32 + r23) * (r32 + r23))
					/ (3 + r11 + r22 - r33));
		}

		// assign signs
		q2 = r32 - r23 < 0 ? -q2 : q2;
		q3 = r13 - r31 < 0 ? -q3 : q3;
		q4 = r21 - r12 < 0 ? -q4 : q4;

		Matrix Q = new Matrix(4, 1);
		Q.set(0, 0, q1);
		Q.set(1, 0, q2);
		Q.set(2, 0, q3);
		Q.set(3, 0, q4);

		return Q;

	}

	public static Matrix setQuatFromMatrix(Matrix mat) {
		double az, ay, ax;
		double ai, aj, ak;
		double si, sj, sk;
		double ci, cj, ck;
		double cy, cc, cs, sc, ss;

		cy = Math.sqrt(mat.get(0, 0) * mat.get(0, 0) + mat.get(1, 0) * mat.get(1, 0));

		if (cy > 0.00000001) {
			ax = Math.atan2(mat.get(2, 1), mat.get(2, 2));
			ay = Math.atan2(-mat.get(2, 0), cy);
			az = Math.atan2(mat.get(1, 0), mat.get(0, 0));
		} else {
			ax = Math.atan2(-mat.get(1, 2), mat.get(1, 1));
			ay = Math.atan2(-mat.get(2, 0), cy);
			az = 0.0;
		}

		ai = ax / 2.0;
		aj = ay / 2.0;
		ak = az / 2.0;

		ci = Math.cos(ai);
		si = Math.sin(ai);
		cj = Math.cos(aj);
		sj = Math.sin(aj);
		ck = Math.cos(ak);
		sk = Math.sin(ak);
		cc = ci * ck;
		cs = ci * sk;
		sc = si * ck;
		ss = si * sk;

		double qx = cj * sc - sj * cs;
		double qy = cj * ss + sj * cc;
		double qz = cj * cs - sj * sc;
		double qw = cj * cc + sj * ss;

		Matrix Q = new Matrix(4, 1);
		Q.set(0, 0, qw);
		Q.set(1, 0, qx);
		Q.set(2, 0, qy);
		Q.set(3, 0, qz);

		return Q;
	}

	public static Matrix OpenCVPnP(ArrayList<Point3D> points3D, ArrayList<Point2D> points2D, Mat rvec, Mat tvec,
			boolean useInitialGuess) {
		ArrayList<Point3> p3Points3D = new ArrayList<Point3>();
		ArrayList<Point> p2Points2D = new ArrayList<Point>();
		for (int i = 0; i < points3D.size(); i++) {
			Point3 point3 = new Point3();
			point3.x = points3D.get(i).getX();
			point3.y = points3D.get(i).getY();
			point3.z = points3D.get(i).getZ();
			p3Points3D.add(point3);
			Point point2 = new Point();
			point2.x = points2D.get(i).getX();
			point2.y = points2D.get(i).getY();
			p2Points2D.add(point2);
		}
		MatOfPoint3f objectPoints = new MatOfPoint3f();
		objectPoints.fromList(p3Points3D);
		MatOfPoint2f imagePoints = new MatOfPoint2f();
		imagePoints.fromList(p2Points2D);
		Mat cameraMatrix = MatrixToMat(CameraIntrinsics.getK());

		Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, new MatOfDouble(), rvec, tvec, useInitialGuess,
				Calib3d.SOLVEPNP_ITERATIVE);

		Mat RMat = new Mat();
		Calib3d.Rodrigues(rvec, RMat);

		Matrix E = Matrix.identity(4, 4);
		E.set(0, 0, RMat.get(0, 0)[0]);
		E.set(0, 1, RMat.get(0, 1)[0]);
		E.set(0, 2, RMat.get(0, 2)[0]);
		E.set(1, 0, RMat.get(1, 0)[0]);
		E.set(1, 1, RMat.get(1, 1)[0]);
		E.set(1, 2, RMat.get(1, 2)[0]);
		E.set(2, 0, RMat.get(2, 0)[0]);
		E.set(2, 1, RMat.get(2, 1)[0]);
		E.set(2, 2, RMat.get(2, 2)[0]);
		E.set(0, 3, tvec.get(0, 0)[0]);
		E.set(1, 3, tvec.get(1, 0)[0]);
		E.set(2, 3, tvec.get(2, 0)[0]);

		return E;
	}

	// perform PnP pose estimation using RANSAC to come up with an initial guess
	// (outRvec and outTvec) and also prune out the outliers from
	// correspondences and points3D
	public static void PnPRANSACPrune(ArrayList<Correspondence2D2D> correspondences, ArrayList<Point3D> points3D,
			Mat outRvec, Mat outTvec) {

		ArrayList<Point3> p3Points3D = new ArrayList<Point3>();
		ArrayList<Point> p2Points2D = new ArrayList<Point>();
		ArrayList<Integer> originalIndices = new ArrayList<Integer>();
		// pl("PnP RANSAC 3D points:");
		for (int i = 0; i < points3D.size(); i++) {
			if (points3D.get(i) != null) {
				Point3 point3 = new Point3();
				point3.x = points3D.get(i).getX();
				point3.y = points3D.get(i).getY();
				point3.z = points3D.get(i).getZ();
				p3Points3D.add(point3);
				// pl(i + ": " + point3.x + ", " + point3.y + ", " + point3.z);
				Point point2 = new Point();
				point2.x = correspondences.get(i).getU2();
				point2.y = correspondences.get(i).getV2();
				p2Points2D.add(point2);
				originalIndices.add(i);
			} else {
				// pl(i + ": null");
			}
		}
		MatOfPoint3f objectPoints = new MatOfPoint3f();
		objectPoints.fromList(p3Points3D);
		MatOfPoint2f imagePoints = new MatOfPoint2f();
		imagePoints.fromList(p2Points2D);
		Mat cameraMatrix = MatrixToMat(CameraIntrinsics.getK());
		Mat inliers = new Mat();

		// perform the RANSAC fitting
		pl("object points size: " + objectPoints.rows());
		pl("image points size: " + imagePoints.rows());
		Calib3d.solvePnPRansac(objectPoints, imagePoints, cameraMatrix, new MatOfDouble(), outRvec, outTvec, false, 100,
				10, 0.95, inliers);

		// prune out the outliers
		// create list of outlier indices, then for each outlier, find it in
		// correspondences and remove from points3D and correspondences
		ArrayList<Integer> outlierIndices = new ArrayList<Integer>();
		for (int i = 0, j = 0; i < p2Points2D.size() && j < inliers.rows(); i++) {
			if (inliers.get(j, 0)[0] > i) {
				outlierIndices.add(i);
				continue;
			}
			j++;
		}

		for (int i = 0; i < outlierIndices.size(); i++) {
			correspondences.remove(originalIndices.get(outlierIndices.get(i)) - i);
			points3D.remove(originalIndices.get(outlierIndices.get(i)) - i);
		}

	}

	public static Matrix PnPNULLIFIED(ArrayList<Point3D> points3D, ArrayList<Point2D> points2D) {
		Matrix B = new Matrix(2 * points3D.size(), 12);

		for (int i = 0; i < points3D.size(); i++) {
			Point3D point3D = points3D.get(i);
			Point2D point2D = points2D.get(i);

			B.set(2 * i, 0, point3D.getX());
			B.set(2 * i, 1, point3D.getY());
			B.set(2 * i, 2, point3D.getZ());
			B.set(2 * i, 3, 1);
			B.set(2 * i, 8, -point2D.getX() * point3D.getX());
			B.set(2 * i, 9, -point2D.getX() * point3D.getY());
			B.set(2 * i, 10, -point2D.getX() * point3D.getZ());
			B.set(2 * i, 11, -point2D.getX());

			B.set(2 * i + 1, 4, point3D.getX());
			B.set(2 * i + 1, 5, point3D.getY());
			B.set(2 * i + 1, 6, point3D.getZ());
			B.set(2 * i + 1, 7, 1);
			B.set(2 * i + 1, 8, -point2D.getY() * point3D.getX());
			B.set(2 * i + 1, 9, -point2D.getY() * point3D.getY());
			B.set(2 * i + 1, 10, -point2D.getY() * point3D.getZ());
			B.set(2 * i + 1, 11, -point2D.getY());
		}
		SingularValueDecomposition svd = B.svd();
		Matrix p = svd.getV().getMatrix(0, 11, 11, 11);
		Matrix pi = new Matrix(3, 4);

		pi.set(0, 0, p.get(0, 0));
		pi.set(0, 1, p.get(1, 0));
		pi.set(0, 2, p.get(2, 0));
		pi.set(0, 3, p.get(3, 0));
		pi.set(1, 0, p.get(4, 0));
		pi.set(1, 1, p.get(5, 0));
		pi.set(1, 2, p.get(6, 0));
		pi.set(1, 3, p.get(7, 0));
		pi.set(2, 0, p.get(8, 0));
		pi.set(2, 1, p.get(9, 0));
		pi.set(2, 2, p.get(10, 0));
		pi.set(2, 3, p.get(11, 0));

		pi = pi.times(1 / pi.getMatrix(2, 2, 0, 2).normF());

		Matrix X = new Matrix(4, 1);
		X.set(0, 0, points3D.get(0).getX());
		X.set(1, 0, points3D.get(0).getY());
		X.set(2, 0, points3D.get(0).getZ());
		X.set(3, 0, 1);
		Matrix x = pi.times(X);
		if (x.get(2, 0) < 0) {
			pi = pi.times(-1);
		}

		Matrix E = CameraIntrinsics.getK().inverse().times(pi);

		E = sanitizeE(E);

		return E;
	}

	/// E1 - 4x4 or 3x4 matrix of camera pose A
	/// E2 - 4x4 or 3x4 matrix of transformation from A to B with unit
	/// translation vector
	/// c - a correspondence between A and B used to estimate scale
	public static double estimateScale(Matrix E1, Matrix E2, Correspondence2D2D c) {
		Matrix K = CameraIntrinsics.getK4x4();

		double oldScore = 0;
		double THRESHOLD = 0.01;
		double scale = 1.0;
		double scaleJump = 10000;
		boolean goRight = true;
		boolean firstPass = true;
		boolean keepGoing = true;
		int i = 0;

		while (keepGoing) {

			// generate fundamental matrix between poses with some scale
			// // - calculate C = R1Inv.times(E1.getMatrix(0,2,3,3)).times(-1)
			Matrix R1Inv = E1.getMatrix(0, 2, 0, 2).inverse();
			Matrix C = R1Inv.times(E1.getMatrix(0, 2, 3, 3)).times(-1);
			Matrix Ch = new Matrix(4, 1);
			Ch.set(0, 0, C.get(0, 0));
			Ch.set(1, 0, C.get(1, 0));
			Ch.set(2, 0, C.get(2, 0));
			Ch.set(3, 0, 1);

			// // - E2 = [R2 | scale * t]
			Matrix E2scaled = E2.copy();
			E2scaled.set(0, 3, E2.get(0, 3) * scale);
			E2scaled.set(1, 3, E2.get(1, 3) * scale);
			E2scaled.set(2, 3, E2.get(2, 3) * scale);

			// // - calculate e' = E2.times(C)
			Matrix Pprime = K.getMatrix(0, 2, 0, 2).times(E2scaled.getMatrix(0, 2, 0, 3));
			Matrix e = Pprime.times(Ch);

			// // - generate [e']x
			Matrix ex = new Matrix(3, 3);
			ex.set(0, 1, -e.get(2, 0));
			ex.set(0, 2, e.get(1, 0));
			ex.set(1, 0, e.get(2, 0));
			ex.set(1, 2, -e.get(0, 0));
			ex.set(2, 0, -e.get(1, 0));
			ex.set(2, 1, e.get(0, 0));

			// // - calculate P+ = E1.inverse()
			Matrix P = K.getMatrix(0, 2, 0, 2).times(E1.getMatrix(0, 2, 0, 3));
			SingularValueDecomposition svd = P.transpose().svd();
			Matrix Splus = new Matrix(3, 3);
			Splus.set(0, 0, 1 / svd.getS().get(0, 0));
			Splus.set(1, 1, 1 / svd.getS().get(1, 1));
			Splus.set(2, 2, 1 / svd.getS().get(2, 2));
			Matrix U = svd.getV();
			Matrix V = svd.getU();
			Matrix PInv = V.times(Splus).times(U.transpose());

			// // - F = [e']x*(P'*P+)
			Matrix F = ex.times(Pprime.times(PInv));
			F = F.times(1 / F.get(2, 2));

			// test a correspondence
			Matrix x = new Matrix(3, 1);
			x.set(0, 0, c.getU1());
			x.set(1, 0, c.getV1());
			x.set(2, 0, 1);

			Matrix xprime = new Matrix(3, 1);
			xprime.set(0, 0, c.getU2());
			xprime.set(1, 0, c.getV2());
			xprime.set(2, 0, 1);

			// score = | x2' * F * x1 |
			double score = Math.abs(xprime.transpose().times(F).times(x).get(0, 0));

			// if the score is good enough, stop
			if (score < THRESHOLD || i >= 100) {
				keepGoing = false;
				continue;
			}

			// if this is the first pass, don't compare with old score
			if (firstPass) {
				firstPass = false;
				oldScore = score;
				continue;
			}

			// if this pose is worse than the last, flip search direction and
			// halve jump distance
			if (score > oldScore) {
				goRight = !goRight;
				if (scaleJump >= 0.25) {
					scaleJump = scaleJump / 2;
				}
			}
			// pl("score: " + score);
			// pl("scale: " + scale);
			// pl("scaleJump: " + scaleJump);

			// prepare a new scale for the next iteration
			scale = goRight ? scale + scaleJump : scale - scaleJump;
			oldScore = score;
			i++;

		}
		return scale;
	}

	public static Frame artificialKeypointFrame(List<Point> points, int width, int height) {
		byte[] r = new byte[width * height];
		byte[] g = new byte[width * height];
		byte[] b = new byte[width * height];
		for (int i = 0; i < width * height; i++) {
			r[i] = 0;
			g[i] = 0;
			b[i] = 0;
		}
		for (Point keypoint : points) {
			if (keypoint.x >= 0 && keypoint.x < width && keypoint.y >= 0 && keypoint.y < height) {
				r[(int) (keypoint.y * width + keypoint.x)] = (byte) 255;
				g[(int) (keypoint.y * width + keypoint.x)] = (byte) 255;
				b[(int) (keypoint.y * width + keypoint.x)] = (byte) 255;
			}
		}
		Frame frame = new Frame(r, g, b, width, height);
		return frame;
	}

	public static Matrix Matrix4fToMatrix(Matrix4f matrix4f) {
		Matrix matrix = new Matrix(4, 4);
		matrix.set(0, 0, matrix4f.m00);
		matrix.set(0, 1, matrix4f.m01);
		matrix.set(0, 2, matrix4f.m02);
		matrix.set(0, 3, matrix4f.m03);
		matrix.set(1, 0, matrix4f.m10);
		matrix.set(1, 1, matrix4f.m11);
		matrix.set(1, 2, matrix4f.m12);
		matrix.set(1, 3, matrix4f.m13);
		matrix.set(2, 0, matrix4f.m20);
		matrix.set(2, 1, matrix4f.m21);
		matrix.set(2, 2, matrix4f.m22);
		matrix.set(2, 3, matrix4f.m23);
		matrix.set(3, 0, matrix4f.m30);
		matrix.set(3, 1, matrix4f.m31);
		matrix.set(3, 2, matrix4f.m32);
		matrix.set(3, 3, matrix4f.m33);
		return matrix;
	}

	public static Matrix4f MatrixToMatrix4f(Matrix matrix) {
		Matrix4f matrix4f = new Matrix4f();
		matrix4f.m00 = (float) matrix.get(0, 0);
		matrix4f.m01 = (float) matrix.get(0, 1);
		matrix4f.m02 = (float) matrix.get(0, 2);
		matrix4f.m03 = (float) matrix.get(0, 3);
		matrix4f.m10 = (float) matrix.get(1, 0);
		matrix4f.m11 = (float) matrix.get(1, 1);
		matrix4f.m12 = (float) matrix.get(1, 2);
		matrix4f.m13 = (float) matrix.get(1, 3);
		matrix4f.m20 = (float) matrix.get(2, 0);
		matrix4f.m21 = (float) matrix.get(2, 1);
		matrix4f.m22 = (float) matrix.get(2, 2);
		matrix4f.m23 = (float) matrix.get(2, 3);
		matrix4f.m30 = (float) matrix.get(3, 0);
		matrix4f.m31 = (float) matrix.get(3, 1);
		matrix4f.m32 = (float) matrix.get(3, 2);
		matrix4f.m33 = (float) matrix.get(3, 3);
		return matrix4f;
	}

	public static ArrayList<Correspondence2D2D> pruneCorrespondences(Matrix fundamentalMatrix,
			ArrayList<Correspondence2D2D> correspondences) {
		ArrayList<Correspondence2D2D> newC = new ArrayList<Correspondence2D2D>();
		for (int i = 0; i < correspondences.size(); i++) {
			Matrix x1 = new Matrix(3, 1);
			x1.set(0, 0, correspondences.get(i).getU1());
			x1.set(1, 0, correspondences.get(i).getV1());
			x1.set(2, 0, 1);

			Matrix x2 = new Matrix(3, 1);
			x2.set(0, 0, correspondences.get(i).getU2());
			x2.set(1, 0, correspondences.get(i).getV2());
			x2.set(2, 0, 1);

			double EPI_THRESH = 0.2;
			double epipolar = x2.transpose().times(fundamentalMatrix).times(x1).get(0, 0);
			// System.out.println("epipolar: " + epipolar);
			if (Math.abs(epipolar) < EPI_THRESH) {
				newC.add(correspondences.get(i));
			}

		}
		return newC;
	}

	public static Matrix triangulate(Matrix E, Matrix pose, Correspondence2D2D c) {

		Matrix Pprime = E.times(pose);
		Pprime = CameraIntrinsics.getK4x4().times(Pprime);

		Matrix P = CameraIntrinsics.getK4x4().times(pose);

		// compute A matrix for Ax = 0
		Matrix row0 = P.getMatrix(2, 2, 0, 3).times(c.getU1()).minus(P.getMatrix(0, 0, 0, 3));
		Matrix row1 = P.getMatrix(2, 2, 0, 3).times(c.getV1()).minus(P.getMatrix(1, 1, 0, 3));
		Matrix row2 = Pprime.getMatrix(2, 2, 0, 3).times(c.getU2()).minus(Pprime.getMatrix(0, 0, 0, 3));
		Matrix row3 = Pprime.getMatrix(2, 2, 0, 3).times(c.getV2()).minus(Pprime.getMatrix(1, 1, 0, 3));

		Matrix A = new Matrix(4, 4);
		A.set(0, 0, row0.get(0, 0));
		A.set(0, 1, row0.get(0, 1));
		A.set(0, 2, row0.get(0, 2));
		A.set(0, 3, row0.get(0, 3));
		A.set(1, 0, row1.get(0, 0));
		A.set(1, 1, row1.get(0, 1));
		A.set(1, 2, row1.get(0, 2));
		A.set(1, 3, row1.get(0, 3));
		A.set(2, 0, row2.get(0, 0));
		A.set(2, 1, row2.get(0, 1));
		A.set(2, 2, row2.get(0, 2));
		A.set(2, 3, row2.get(0, 3));
		A.set(3, 0, row3.get(0, 0));
		A.set(3, 1, row3.get(0, 1));
		A.set(3, 2, row3.get(0, 2));
		A.set(3, 3, row3.get(0, 3));

		SingularValueDecomposition svd = A.svd();
		Matrix X = svd.getV().getMatrix(0, 3, 3, 3);
		X = X.times(1.0 / X.get(3, 0));
		// System.out.println("X");
		// X.print(5, 4);
		return X;
	}

	public static Rt selectEssentialSolution(EssentialDecomposition decomp, Matrix pose,
			ArrayList<Correspondence2D2D> correspondences) {
		Rt rt = new Rt();
		rt.setR(decomp.getR1());
		rt.setT(decomp.getT());

		// get scale for each combination
		double scale = 1;

		// set up extrinsic matrices (both possible options)
		Matrix E1 = Matrix.identity(4, 4);
		Matrix E2 = Matrix.identity(4, 4);
		Matrix E3 = Matrix.identity(4, 4);
		Matrix E4 = Matrix.identity(4, 4);
		E1.set(0, 0, decomp.getR1().get(0, 0));
		E1.set(0, 1, decomp.getR1().get(0, 1));
		E1.set(0, 2, decomp.getR1().get(0, 2));
		E1.set(1, 0, decomp.getR1().get(1, 0));
		E1.set(1, 1, decomp.getR1().get(1, 1));
		E1.set(1, 2, decomp.getR1().get(1, 2));
		E1.set(2, 0, decomp.getR1().get(2, 0));
		E1.set(2, 1, decomp.getR1().get(2, 1));
		E1.set(2, 2, decomp.getR1().get(2, 2));
		E1.set(0, 3, decomp.getT().get(0, 0) * scale);
		E1.set(1, 3, decomp.getT().get(1, 0) * scale);
		E1.set(2, 3, decomp.getT().get(2, 0) * scale);

		E2.set(0, 0, decomp.getR2().get(0, 0));
		E2.set(0, 1, decomp.getR2().get(0, 1));
		E2.set(0, 2, decomp.getR2().get(0, 2));
		E2.set(1, 0, decomp.getR2().get(1, 0));
		E2.set(1, 1, decomp.getR2().get(1, 1));
		E2.set(1, 2, decomp.getR2().get(1, 2));
		E2.set(2, 0, decomp.getR2().get(2, 0));
		E2.set(2, 1, decomp.getR2().get(2, 1));
		E2.set(2, 2, decomp.getR2().get(2, 2));
		E2.set(0, 3, decomp.getT().get(0, 0) * scale);
		E2.set(1, 3, decomp.getT().get(1, 0) * scale);
		E2.set(2, 3, decomp.getT().get(2, 0) * scale);

		E3 = E1.copy();
		E3.set(0, 3, -E3.get(0, 3));
		E3.set(1, 3, -E3.get(1, 3));
		E3.set(2, 3, -E3.get(2, 3));

		E4 = E2.copy();
		E4.set(0, 3, -E4.get(0, 3));
		E4.set(1, 3, -E4.get(1, 3));
		E4.set(2, 3, -E4.get(2, 3));

		pl("E1:");
		E1.print(15, 5);
		pl("E2:");
		E2.print(15, 5);
		pl("E3:");
		E3.print(15, 5);
		pl("E4:");
		E4.print(15, 5);

		// pick a correspondence
		Correspondence2D2D c = correspondences.get(0);

		pl("correspondence:");
		pl(c.getU1() + ", " + c.getV1());
		pl(c.getU2() + ", " + c.getV2());

		Matrix X1 = triangulate(E1, pose, c);
		Matrix X2 = triangulate(E2, pose, c);
		Matrix X3 = triangulate(E3, pose, c);
		Matrix X4 = triangulate(E4, pose, c);

		pl("X1: ");
		X1.print(10, 5);
		pl("X2: ");
		X2.print(10, 5);
		pl("X3: ");
		X3.print(10, 5);
		pl("X4: ");
		X4.print(10, 5);

		// fix this
		Matrix b1 = CameraIntrinsics.getK4x4().times(E1).times(pose).times(X1);
		Matrix b2 = CameraIntrinsics.getK4x4().times(E2).times(pose).times(X2);
		Matrix b3 = CameraIntrinsics.getK4x4().times(E3).times(pose).times(X3);
		Matrix b4 = CameraIntrinsics.getK4x4().times(E4).times(pose).times(X4);
		int numSet = 0;

		pl("b1: ");
		b1.print(15, 5);
		pl("b2: ");
		b2.print(15, 5);
		pl("b3: ");
		b3.print(15, 5);
		pl("b4: ");
		b4.print(15, 5);

		if (b1.get(2, 0) > 0) {
			Matrix a1 = pose.times(X1);
			pl("a1:");
			a1.print(15, 5);
			if (a1.get(2, 0) > 0) {
				rt.setR(decomp.getR1());
				rt.setT(decomp.getT().times(scale));
				numSet++;
			}
		}

		if (b2.get(2, 0) > 0) {
			Matrix a2 = pose.times(X2);
			pl("a2:");
			a2.print(15, 5);
			if (a2.get(2, 0) > 0) {
				rt.setR(decomp.getR2());
				rt.setT(decomp.getT().times(scale));
				numSet++;
			}
		}

		if (b3.get(2, 0) > 0) {
			Matrix a3 = pose.times(X3);
			pl("a3:");
			a3.print(15, 5);
			if (a3.get(2, 0) > 0) {
				rt.setR(decomp.getR1());
				rt.setT(decomp.getT().times(scale).times(-1));
				numSet++;
			}
		}

		if (b4.get(2, 0) > 0) {
			Matrix a4 = pose.times(X4);
			pl("a4:");
			a4.print(15, 5);
			if (a4.get(2, 0) > 0) {
				rt.setR(decomp.getR2());
				rt.setT(decomp.getT().times(scale).times(-1));
				numSet++;
			}
		}

		pl("numSet = " + numSet);

		pl("chosen R: ");
		rt.getR().print(15, 5);
		pl("chosen t: ");
		rt.getT().print(15, 5);

		return rt;
	}

	public static Rt selectHomographySolution(List<Mat> rotations, List<Mat> translations, Matrix pose,
			ArrayList<Correspondence2D2D> correspondences) {
		Rt rt = new Rt();

		Matrix R1 = MatToMatrix(rotations.get(0));
		Matrix R2 = MatToMatrix(rotations.get(1));
		Matrix R3 = MatToMatrix(rotations.get(2));
		Matrix R4 = MatToMatrix(rotations.get(3));

		Matrix t1 = MatToMatrix(translations.get(0));
		Matrix t2 = MatToMatrix(translations.get(1));
		Matrix t3 = MatToMatrix(translations.get(2));
		Matrix t4 = MatToMatrix(translations.get(3));

		rt.setR(R1);
		rt.setT(t1);

		// set up extrinsic matrices (all possible options)
		Matrix E1 = Matrix.identity(4, 4);
		Matrix E2 = Matrix.identity(4, 4);
		Matrix E3 = Matrix.identity(4, 4);
		Matrix E4 = Matrix.identity(4, 4);

		E1.set(0, 0, R1.get(0, 0));
		E1.set(0, 1, R1.get(0, 1));
		E1.set(0, 2, R1.get(0, 2));
		E1.set(1, 0, R1.get(1, 0));
		E1.set(1, 1, R1.get(1, 1));
		E1.set(1, 2, R1.get(1, 2));
		E1.set(2, 0, R1.get(2, 0));
		E1.set(2, 1, R1.get(2, 1));
		E1.set(2, 2, R1.get(2, 2));
		E1.set(0, 3, t1.get(0, 0));
		E1.set(1, 3, t1.get(1, 0));
		E1.set(2, 3, t1.get(2, 0));

		E2.set(0, 0, R2.get(0, 0));
		E2.set(0, 1, R2.get(0, 1));
		E2.set(0, 2, R2.get(0, 2));
		E2.set(1, 0, R2.get(1, 0));
		E2.set(1, 1, R2.get(1, 1));
		E2.set(1, 2, R2.get(1, 2));
		E2.set(2, 0, R2.get(2, 0));
		E2.set(2, 1, R2.get(2, 1));
		E2.set(2, 2, R2.get(2, 2));
		E2.set(0, 3, t2.get(0, 0));
		E2.set(1, 3, t2.get(1, 0));
		E2.set(2, 3, t2.get(2, 0));

		E3.set(0, 0, R3.get(0, 0));
		E3.set(0, 1, R3.get(0, 1));
		E3.set(0, 2, R3.get(0, 2));
		E3.set(1, 0, R3.get(1, 0));
		E3.set(1, 1, R3.get(1, 1));
		E3.set(1, 2, R3.get(1, 2));
		E3.set(2, 0, R3.get(2, 0));
		E3.set(2, 1, R3.get(2, 1));
		E3.set(2, 2, R3.get(2, 2));
		E3.set(0, 3, t3.get(0, 0));
		E3.set(1, 3, t3.get(1, 0));
		E3.set(2, 3, t3.get(2, 0));

		E4.set(0, 0, R4.get(0, 0));
		E4.set(0, 1, R4.get(0, 1));
		E4.set(0, 2, R4.get(0, 2));
		E4.set(1, 0, R4.get(1, 0));
		E4.set(1, 1, R4.get(1, 1));
		E4.set(1, 2, R4.get(1, 2));
		E4.set(2, 0, R4.get(2, 0));
		E4.set(2, 1, R4.get(2, 1));
		E4.set(2, 2, R4.get(2, 2));
		E4.set(0, 3, t4.get(0, 0));
		E4.set(1, 3, t4.get(1, 0));
		E4.set(2, 3, t4.get(2, 0));

		pl("E1:");
		E1.print(15, 5);
		pl("E2:");
		E2.print(15, 5);
		pl("E3:");
		E3.print(15, 5);
		pl("E4:");
		E4.print(15, 5);

		int[] scores = { 0, 0, 0, 0 };
		double[] reprojErrors = { 0, 0, 0, 0 };

		for (Correspondence2D2D c : correspondences) {

			// triangulated points
			Matrix X1 = triangulate(E1, pose, c);
			Matrix X2 = triangulate(E2, pose, c);
			Matrix X3 = triangulate(E3, pose, c);
			Matrix X4 = triangulate(E4, pose, c);

			// reprojected to second frame
			Matrix b1 = CameraIntrinsics.getK4x4().times(E1).times(pose).times(X1);
			Matrix b2 = CameraIntrinsics.getK4x4().times(E2).times(pose).times(X2);
			Matrix b3 = CameraIntrinsics.getK4x4().times(E3).times(pose).times(X3);
			Matrix b4 = CameraIntrinsics.getK4x4().times(E4).times(pose).times(X4);

			// get reprojection errors
			Matrix b1Normalized = b1.times(1 / b1.get(2, 0)).getMatrix(0, 1, 0, 0);
			Matrix b2Normalized = b2.times(1 / b2.get(2, 0)).getMatrix(0, 1, 0, 0);
			Matrix b3Normalized = b3.times(1 / b3.get(2, 0)).getMatrix(0, 1, 0, 0);
			Matrix b4Normalized = b4.times(1 / b4.get(2, 0)).getMatrix(0, 1, 0, 0);
			Matrix trueProj = new Matrix(2, 1);
			trueProj.set(0, 0, c.getU2());
			trueProj.set(1, 0, c.getV2());
			reprojErrors[0] += trueProj.minus(b1Normalized).normF();
			reprojErrors[1] += trueProj.minus(b2Normalized).normF();
			reprojErrors[2] += trueProj.minus(b3Normalized).normF();
			reprojErrors[3] += trueProj.minus(b4Normalized).normF();

			if (b1.get(2, 0) > 0) {
				Matrix a1 = pose.times(X1);
				if (a1.get(2, 0) > 0) {
					rt.setR(R1);
					rt.setT(t1);
					scores[0]++;
				}
			}

			if (b2.get(2, 0) > 0) {
				Matrix a2 = pose.times(X2);
				if (a2.get(2, 0) > 0) {
					rt.setR(R2);
					rt.setT(t2);
					scores[1]++;
				}
			}

			if (b3.get(2, 0) > 0) {
				Matrix a3 = pose.times(X3);
				if (a3.get(2, 0) > 0) {
					rt.setR(R3);
					rt.setT(t3);
					scores[2]++;
				}
			}

			if (b4.get(2, 0) > 0) {
				Matrix a4 = pose.times(X4);
				if (a4.get(2, 0) > 0) {
					rt.setR(R4);
					rt.setT(t4);
					scores[3]++;
				}
			}
		}

		// narrow down options based on chirality (should have 2 hypotheses
		// remaining)
		ArrayList<Integer> bestHypothesesInd = new ArrayList<Integer>();
		double avgScore = (scores[0] + scores[1] + scores[2] + scores[3]) / 4.0;

		for (int i = 0; i < scores.length; i++) {
			int score = scores[i];
			if (score > avgScore) {
				bestHypothesesInd.add(i);
			}
		}

		// pick hypothesis based on reprojection error
		int lowestReprojInd = bestHypothesesInd.get(0);
		for (int i = 1; i < bestHypothesesInd.size(); i++) {
			if (reprojErrors[bestHypothesesInd.get(i)] < reprojErrors[lowestReprojInd]) {
				lowestReprojInd = bestHypothesesInd.get(i);
			}
		}

		// finally, set the correct decomposition
		if (lowestReprojInd == 0) {
			rt.setR(R1);
			rt.setT(t1);
		} else if (lowestReprojInd == 1) {
			rt.setR(R2);
			rt.setT(t2);
		} else if (lowestReprojInd == 2) {
			rt.setR(R3);
			rt.setT(t3);
		} else if (lowestReprojInd == 3) {
			rt.setR(R4);
			rt.setT(t4);
		}

		pl("scores:");
		for (int score : scores) {
			pl(score);
		}

		pl("reprojection errors:");
		for (double error : reprojErrors) {
			pl(error);
		}

		pl("chosen R: ");
		rt.getR().print(15, 5);
		pl("chosen t: ");
		rt.getT().print(15, 5);

		return rt;
	}

	public static Matrix transformPose(Matrix R, Matrix t, Matrix pose) {
		// make homogeneous
		Matrix homogeneousE = Matrix.identity(4, 4);
		homogeneousE.set(0, 0, R.get(0, 0));
		homogeneousE.set(0, 1, R.get(0, 1));
		homogeneousE.set(0, 2, R.get(0, 2));
		homogeneousE.set(1, 0, R.get(1, 0));
		homogeneousE.set(1, 1, R.get(1, 1));
		homogeneousE.set(1, 2, R.get(1, 2));
		homogeneousE.set(2, 0, R.get(2, 0));
		homogeneousE.set(2, 1, R.get(2, 1));
		homogeneousE.set(2, 2, R.get(2, 2));
		homogeneousE.set(0, 3, t.get(0, 0));
		homogeneousE.set(1, 3, t.get(1, 0));
		homogeneousE.set(2, 3, t.get(2, 0));

		Matrix newPose = homogeneousE.times(pose);
		return newPose;
	}

	public static EssentialDecomposition decomposeEssentialMat(Matrix essentialMatrix) {

		EssentialDecomposition decomp = new EssentialDecomposition();
		SingularValueDecomposition svd = essentialMatrix.svd();
		Matrix U = svd.getU();
		Matrix V = svd.getV();

		if (U.det() < 0) {
			U = U.times(-1);
		}
		if (V.transpose().det() < 0) {
			V = V.times(-1);
		}

		Matrix W = new Matrix(3, 3);
		W.set(0, 1, -1);
		W.set(1, 0, 1);
		W.set(2, 2, 1);

		Matrix t = U.getMatrix(0, 2, 2, 2);
		Matrix R1 = U.times(W.transpose()).times(V.transpose());
		Matrix R2 = U.times(W).times(V.transpose());

		decomp.setR1(R1);
		decomp.setR2(R2);
		decomp.setT(t);

		return decomp;
	}

	public static Mat frameToMat(Frame frame) {
		Mat mat = new Mat(frame.getHeight(), frame.getWidth(), CvType.CV_8UC3);
		for (int row = 0; row < frame.getHeight(); row++) {
			for (int col = 0; col < frame.getWidth(); col++) {

				byte GREY = 0;
				byte R = 0;
				byte G = 0;
				byte B = 0;

				if (frame.getR() == null) {
					GREY = frame.getGrey()[frame.getWidth() * row + col];
					R = GREY;
					G = GREY;
					B = GREY;
				} else {

					B = frame.getB()[frame.getWidth() * row + col];
					G = frame.getG()[frame.getWidth() * row + col];
					R = frame.getR()[frame.getWidth() * row + col];
				}

				byte[] rgb = { B, G, R };
				mat.put(row, col, rgb);
			}
		}
		return mat;
	}

	public static Mat MatrixToMat(Matrix matrix) {
		Mat mat = new Mat(matrix.getRowDimension(), matrix.getColumnDimension(), CvType.CV_32F);
		for (int row = 0; row < matrix.getRowDimension(); row++) {
			for (int col = 0; col < matrix.getColumnDimension(); col++) {
				mat.put(row, col, matrix.get(row, col));
			}
		}
		return mat;
	}

	public static Matrix MatToMatrix(Mat mat) {
		Matrix matrix = new Matrix(mat.rows(), mat.cols());

		for (int row = 0; row < mat.rows(); row++) {
			for (int col = 0; col < mat.cols(); col++) {
				matrix.set(row, col, mat.get(row, col)[0]);
			}
		}

		return matrix;
	}

	public static void boxHighlight(Frame frame, MatOfKeyPoint keypoints, byte[][] RGB, int boxSize) {
		List<KeyPoint> keypointList = keypoints.toList();
		for (KeyPoint keypoint : keypointList) {
			if (true) {
				boxHighlight(frame, keypoint, RGB, boxSize * (int) Math.pow(1.5, keypoint.octave));
			}
		}
	}

	public static void boxHighlight(Frame frame, MatOfKeyPoint keypoints) {
		byte[][] RGB = { { 0, 0, (byte) 255 }, { (byte) 255, 0, 0 }, { 0, (byte) 255, 0 },
				{ (byte) 255, 0, (byte) 255 } };
		boxHighlight(frame, keypoints, RGB, 21);
	}

	public static void boxHighlight(Frame frame, MatOfKeyPoint keypoints, int boxSize) {

		byte[][] RGB = { { 0, 0, (byte) 255 }, { (byte) 255, 0, 0 }, { 0, (byte) 255, 0 },
				{ (byte) 255, 0, (byte) 255 } };
		boxHighlight(frame, keypoints, RGB, boxSize);
	}

	public static ArrayList<Byte[]> generateColors(Mat descriptors) {
		ArrayList<Byte[]> colors = new ArrayList<Byte[]>();
		for (int i = 0; i < descriptors.rows(); i++) {
			colors.add(getColor(descriptors.row(i)));
		}
		return colors;
	}

	public static Byte[] getColor(Mat descriptor) {
		double r = 0;
		double g = 0;
		double b = 0;
		for (int i = 0; i < descriptor.cols(); i++) {
			if (i / 11 == 0) {
				r += descriptor.get(0, i)[0];
			} else if (i / 11 == 1) {
				g += descriptor.get(0, i)[0];
			} else {
				b += descriptor.get(0, i)[0];
			}
		}
		r /= 2805.0;
		g /= 2805.0;
		b /= 2550.0;

		r *= 255;
		g *= 255;
		b *= 255;
		Byte[] rgb = { (byte) r, (byte) g, (byte) b };
		return rgb;
	}

	public static void boxHighlight(Frame frame, MatOfKeyPoint keypoints, Mat descriptors, int boxSize) {
		ArrayList<Byte[]> colors = generateColors(descriptors);
		boxHighlightColored(frame, keypoints, colors, boxSize);
	}

	public static void boxHighlightColored(Frame frame, MatOfKeyPoint keypoints, ArrayList<Byte[]> keypointColors,
			int boxSize) {
		List<KeyPoint> keypointList = keypoints.toList();
		for (int i = 0; i < keypointList.size(); i++) {
			byte[] rgb = { keypointColors.get(i)[0], keypointColors.get(i)[1], keypointColors.get(i)[2] };
			boxHighlight(frame, (int) keypointList.get(i).pt.x, (int) keypointList.get(i).pt.y, rgb, boxSize);
		}
	}

	public static void boxHighlight(Frame frame, KeyPoint keypoint, byte[][] RGB, int boxSize) {

		int x = (int) keypoint.pt.x;
		int y = (int) keypoint.pt.y;
		byte[] color = getRGB(RGB, keypoint.octave);
		boxHighlight(frame, x, y, color, boxSize);

	}

	public static void boxHighlight(Frame frame, List<Correspondence2D2D> correspondences, int boxSize) {
		for (int i = 0; i < correspondences.size(); i++) {
			Correspondence2D2D c = correspondences.get(i);
			byte[] rgb = { (byte) (c.getU1().intValue() % 256), (byte) (c.getV1().intValue() % 256), 0 };
			boxHighlight(frame, c.getU2().intValue(), c.getV2().intValue(), rgb, boxSize);
		}
	}

	public static void boxHighlight(Frame frame, int x, int y, byte[] RGB, int boxSize) {
		boxHighlight(frame, x, y, RGB, boxSize, boxSize);
	}

	public static void boxHighlight(Frame frame, int x, int y, byte[] RGB, int width, int height) {

		int thickness = 1;
		int wOffset = width / 2;
		int hOffset = height / 2;
		byte[] color = RGB;

		// for every pixel in the box area around the point
		for (int row = 0; row < height; row++) {
			for (int col = 0; col < width; col++) {
				int pixelPosX = col + x - wOffset;
				int pixelPosY = row + y - hOffset;

				// do we color it?
				// check for boundaries of image
				if (pixelPosX > 0 && pixelPosX < frame.getWidth() && pixelPosY > 0 && pixelPosY < frame.getHeight()) {

					// check for distance from barrier
					if (row < thickness || row >= height - thickness || col < thickness || col >= width - thickness) {

						// paint the pixel
						frame.getGrey()[frame.getWidth() * pixelPosY
								+ pixelPosX] = (byte) ((color[0] + color[1] + color[2]) / 3);

						if (frame.getR() != null) {
							frame.getR()[frame.getWidth() * pixelPosY + pixelPosX] = color[0];
							frame.getG()[frame.getWidth() * pixelPosY + pixelPosX] = color[1];
							frame.getB()[frame.getWidth() * pixelPosY + pixelPosX] = color[2];
						}

					}

				}
			}
		}

	}

	public static byte[] getRGB(byte[][] RGB, int octave) {
		if (octave >= RGB.length) {
			return RGB[RGB.length - 1];
		} else {
			return RGB[octave];
		}
	}

	public static void nonMaximumSuppression(MatOfKeyPoint keypoints, double patchSize) {
		List<KeyPoint> keypointList = new ArrayList<KeyPoint>(keypoints.toList());
		for (int i = 0; i < keypointList.size(); i++) {
			// find all keypoints sufficiently overlapping this one
			List<KeyPoint> interestPoints = new ArrayList<KeyPoint>();
			interestPoints.add(keypointList.get(i));

			for (int j = i + 1; j < keypointList.size(); j++) {
				if (keypointList.get(i).octave == keypointList.get(j).octave && getDistance(keypointList.get(j).pt,
						keypointList.get(i).pt) < patchSize * Math.pow(2, keypointList.get(j).octave)) {
					interestPoints.add(keypointList.get(j));
				}
			}

			if (interestPoints.size() > 1) {
				// remove all interest points
				for (int k = 1; k < interestPoints.size(); k++) {
					keypointList.remove(interestPoints.get(k));
				}

				// insert centered point
				keypointList.set(i, getStrongest(interestPoints));
			}

		}
		keypoints.fromList(keypointList);
	}

	public static double getDistance(Point a, Point b) {
		double x = a.x - b.x;
		double y = a.y - b.y;
		return Math.sqrt(x * x + y * y);
	}

	public static KeyPoint getCentroid(List<KeyPoint> keypoints) {
		float sumX = 0;
		float sumY = 0;
		for (KeyPoint keypoint : keypoints) {
			sumX += keypoint.pt.x;
			sumY += keypoint.pt.y;
		}
		sumX = sumX / keypoints.size();
		sumY = sumY / keypoints.size();
		KeyPoint centered = new KeyPoint(sumX, sumY, keypoints.get(0).size, keypoints.get(0).angle,
				keypoints.get(0).response, keypoints.get(0).octave, keypoints.get(0).class_id);
		return centered;
	}

	public static KeyPoint getStrongest(List<KeyPoint> keypoints) {
		int strongestIndex = 0;
		for (int i = 0; i < keypoints.size(); i++) {
			if (keypoints.get(strongestIndex).response < keypoints.get(i).response) {
				strongestIndex = i;
			}
		}
		return keypoints.get(strongestIndex);
	}

	public static void pruneOctaves(MatOfKeyPoint keypoints, List<Integer> octavesToKeep) {
		List<KeyPoint> keypointList = new ArrayList<KeyPoint>(keypoints.toList());
		for (int i = 0; i < keypointList.size(); i++) {
			if (!octavesToKeep.contains(keypointList.get(i).octave)) {
				keypointList.remove(i);
				i--;
			}
		}
		keypoints.fromList(keypointList);
	}

	public static void pruneKeypoints(MatOfKeyPoint keypoints, int responseThreshold) {
		List<KeyPoint> keypointList = keypoints.toList();
		List<KeyPoint> newKeypoints = new ArrayList<KeyPoint>();
		for (int i = 0; i < keypointList.size(); i++) {
			if (keypointList.get(i).response >= responseThreshold) {
				newKeypoints.add(keypointList.get(i));
			}
		}
		keypoints.fromList(newKeypoints);
	}

	// given a keyframe, go through its map points and project its 3D points
	// onto the image
	public static void projectMapPoints(KeyFrame keyframe, Frame frame, Pose pose, Matrix K4x4) {
		Matrix E = pose.getHomogeneousMatrix();
		byte[] RGB = { (byte) 255, 0, (byte) 127 };
		int boxSize = 3;
		for (int i = 0; i < keyframe.getMapPoints().size(); i++) {
			Point3D point = keyframe.getMapPoints().get(i).getPoint();

			if (point == null) {
				continue;
			}

			// project 3D point to this pose
			Matrix point3D = new Matrix(4, 1);
			point3D.set(0, 0, point.getX());
			point3D.set(1, 0, point.getY());
			point3D.set(2, 0, point.getZ());
			point3D.set(3, 0, 1);

			Matrix projection = K4x4.times(E).times(point3D);
			projection = projection.times(1 / projection.get(2, 0));

			// if point is on screen, paint it
			int x = (int) projection.get(0, 0);
			int y = (int) projection.get(1, 0);
			if (x < frame.getWidth() && x >= 0 && y < frame.getHeight() && y >= 0) {
				boxHighlight(frame, x, y, RGB, boxSize);
			}
		}
	}

	public static byte[] dullColor(byte[] color) {
		double avg = 0;
		for (int i = 0; i < color.length; i++) {
			avg += mod(color[i], 256);
		}
		avg /= color.length;
		byte[] newColor = new byte[color.length];
		for (int i = 0; i < color.length; i++) {
			double diff = avg - mod(color[i], 256);
			newColor[i] = (byte) (mod(color[i], 256) + (diff * 2 / 3));
		}
		return newColor;
	}

	public static void trackActiveSearch(Frame frame, ArrayList<ActiveSearchData> searchData, long frameNum) {
		byte[] RGB = { (byte) 255, (byte) 255, 0 };
		trackActiveSearch(frame, searchData, frameNum, RGB);
	}

	public static void trackActiveSearch(Frame frame, ArrayList<ActiveSearchData> searchData, long frameNum,
			byte[] RGB) {
		for (int i = 0; i < searchData.size(); i++) {
			ActiveSearchData s = searchData.get(i);

			long framesSinceSeen = frameNum - s.getLastFrameObserved();
			byte[] color = RGB;
			if (framesSinceSeen > 10) {
				color = dullColor(color);
			}

			int x1 = (int) s.getLastLocation().getX();
			int y1 = (int) s.getLastLocation().getY();
			int x2 = framesSinceSeen > 10 ? (int) (x1 + s.getDx() * 10) : (int) (x1 + s.getDx() * framesSinceSeen);
			int y2 = framesSinceSeen > 10 ? (int) (y1 + s.getDy() * 10) : (int) (y1 + s.getDy() * framesSinceSeen);

			paintLine(frame, x1, y1, x2, y2, color);

		}
	}

	public static void trackActiveSearchN(Frame frame, ArrayList<ActiveSearchData> searchData, long frameNum,
			byte[] RGB) {
		for (int i = 0; i < searchData.size(); i++) {
			ActiveSearchData s = searchData.get(i);

			long framesSinceSeen = frameNum - s.getLastFrameObserved();
			byte[] color = RGB;
			if (framesSinceSeen > 10) {
				color = dullColor(color);
			}

			int x1 = (int) s.getLastLocation().getX();
			int y1 = (int) s.getLastLocation().getY();
			int x2 = framesSinceSeen > 10 ? (int) (x1 + s.getDxN() * 10) : (int) (x1 + s.getDxN() * framesSinceSeen);
			int y2 = framesSinceSeen > 10 ? (int) (y1 + s.getDyN() * 10) : (int) (y1 + s.getDyN() * framesSinceSeen);

			paintLine(frame, x1, y1, x2, y2, color);

		}
	}

	public static void trackCorrespondences(Frame frame, ArrayList<Correspondence2D2D> correspondences) {
		byte[] RGB = { 0, (byte) 255, 0 };
		trackCorrespondences(frame, correspondences, RGB);
	}

	public static void trackCorrespondences(Frame frame, ArrayList<Correspondence2D2D> correspondences, byte[] RGB) {
		for (int i = 0; i < correspondences.size(); i++) {
			Correspondence2D2D c = correspondences.get(i);
			int x1 = c.getU1().intValue();
			int y1 = c.getV1().intValue();
			int x2 = c.getU2().intValue();
			int y2 = c.getV2().intValue();

			paintLine(frame, x1, y1, x2, y2, RGB);

		}
	}

	public static void paintLine(Frame frame, int x1, int y1, int x2, int y2, byte[] RGB) {

		int currentX = x1;
		int currentY = y1;

		int xDiff = x2 > x1 ? 1 : -1;
		int yDiff = y2 > y1 ? 1 : -1;

		int y2minusy1 = y2 - y1;
		int x2minusx1 = x2 - x1;
		int x2y1 = x2 * y1;
		int y2x1 = y2 * x1;
		double baseDist = Math.sqrt(y2minusy1 * y2minusy1 + x2minusx1 * x2minusx1);

		boolean keepGoing = true;
		while (keepGoing) {

			if (currentX == x2 && currentY == y2) {
				keepGoing = false;
			}

			paintPixel(frame, currentX, currentY, RGB, 1);

			// evaluate 3 hypotheses and pick best one
			double distX = Math.abs(y2minusy1 * (currentX + xDiff) - x2minusx1 * currentY + x2y1 - y2x1) / baseDist;
			double distY = Math.abs(y2minusy1 * currentX - x2minusx1 * (currentY + yDiff) + x2y1 - y2x1) / baseDist;
			double distXY = Math.abs(y2minusy1 * (currentX + xDiff) - x2minusx1 * (currentY + yDiff) + x2y1 - y2x1)
					/ baseDist;

			if (distXY < distX && distXY < distY) {
				currentX += xDiff;
				currentY += yDiff;
			} else if (distX < distY) {
				currentX += xDiff;
			} else {
				currentY += yDiff;
			}
		}

	}

	public static void paintPixel(Frame frame, int x, int y, byte[] RGB, int thickness) {
		for (int i = x - thickness; i < x + thickness; i++) {
			for (int j = y - thickness; j < y + thickness; j++) {

				// if this point is on screen, paint it
				if (i >= 0 && i < frame.getWidth() && j >= 0 && j < frame.getHeight()) {
					frame.getR()[frame.getWidth() * j + i] = RGB[0];
					frame.getG()[frame.getWidth() * j + i] = RGB[1];
					frame.getB()[frame.getWidth() * j + i] = RGB[2];
				}

			}
		}
	}

	public static Double min(ArrayList<Double> list) {
		Double min = list.get(0);
		for (int i = 1; i < list.size(); i++) {
			if (min > list.get(i)) {
				min = list.get(i);
			}
		}
		return min;
	}

	public static Double max(ArrayList<Double> list) {
		Double max = list.get(0);
		for (int i = 1; i < list.size(); i++) {
			if (max < list.get(i)) {
				max = list.get(i);
			}
		}
		return max;
	}

	public static Double sum(ArrayList<Double> list) {
		Double sum = 0.0;
		for (int i = 0; i < list.size(); i++) {
			sum += list.get(i);
		}
		return sum;
	}

	public static Double mean(ArrayList<Double> list) {
		return sum(list) / list.size();
	}

	public static Double variance(ArrayList<Double> list) {
		Double avg = mean(list);
		Double variance = 0.0;
		for (int i = 0; i < list.size(); i++) {
			variance += Math.pow(list.get(i) - avg, 2);
		}
		return variance / avg;
	}

	public static Double stdDev(ArrayList<Double> list) {
		return Math.sqrt(variance(list));
	}

	public static Double descriptorDistance(Mat desc1, Mat desc2) {
		Double sum = 0.0;
		for (int i = 0; i < desc1.cols(); i++) {
			sum += Math.pow(desc1.get(0, i)[0] - desc2.get(0, i)[0], 2);
		}
		return Math.sqrt(sum);
	}

	public static double mod(double a, double b) {
		return ((a % b) + b) % b;
	}

	public static void p(Object s) {
		System.out.print(s);
	}

	public static void pl(Object s) {
		System.out.println(s);
	}
}
