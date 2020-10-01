import java.awt.Dimension;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.swing.SwingUtilities;

import org.ddogleg.optimization.lm.ConfigLevenbergMarquardt;
import org.opencv.core.Core;

import ARPipeline.ARPipeline;
import ARPipeline.ARUtils;
import ARPipeline.MockPipeline;
import ARPipeline.MockPointData;
import ARPipeline.Point2D;
import ARPipeline.Point3D;
import ARPipeline.Pose;
import ARPipeline.SingletonFrameBuffer;
import ARPipeline.SingletonPoseBuffer;
import Jama.Matrix;
import boofcv.abst.geo.bundle.BundleAdjustment;
import boofcv.abst.geo.bundle.ScaleSceneStructure;
import boofcv.abst.geo.bundle.SceneStructureMetric;
import boofcv.factory.geo.ConfigBundleAdjustment;
import boofcv.factory.geo.FactoryMultiView;
import boofcv.gui.image.ShowImages;
import boofcv.io.geo.CodecBundleAdjustmentInTheLarge;
import boofcv.visualize.PointCloudViewer;
import boofcv.visualize.SingleAxisRgb;
import boofcv.visualize.VisualizeData;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.metric.UtilAngle;
import georegression.struct.EulerType;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;

public class ARBootstrapper {

	String SAMPLE_PATH = "src/samples/";
	String filename = SAMPLE_PATH + "roomFloor01_270.avi";

	public ARBootstrapper() {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	public void start() {

		// OfflineFrameBuffer ofb = new OfflineFrameBuffer(filename, false);
		SingletonPoseBuffer spb = new SingletonPoseBuffer();
		SingletonFrameBuffer sfb = new SingletonFrameBuffer();
		// ARPipeline pipeline = new TestPipeline(ofb, spb, sfb);
		ARPipeline pipeline = new MockPipeline(sfb, spb, sfb);
		OpenGLARDisplay ARDisplay = new OpenGLARDisplay(sfb, spb);

		pipeline.start();
		ARDisplay.displayLoop();

		println("Done.");
	}

	public static void println(Object obj) {
		System.out.println(obj);
	}

	public static void main(String[] args) {
		ARBootstrapper arBootstrapper = new ARBootstrapper();
		// arBootstrapper.start();
		// arBootstrapper.tests();
		try {
			arBootstrapper.boof();
		} catch (Exception e) {
			pl("error in boof");
			e.printStackTrace();
		}

	}

	public void tests() {

		MockPointData mock = new MockPointData();
		ArrayList<Pose> cameras = new ArrayList<Pose>();
		ArrayList<Point3D> point3Ds = new ArrayList<Point3D>();
		ArrayList<ArrayList<Point2D>> observations = new ArrayList<ArrayList<Point2D>>();

		int START_FRAME = 0;
		int END_FRAME = 50;

		// poses
		Matrix R1 = mock.getR(START_FRAME);
		Matrix IC1 = mock.getIC(START_FRAME);
		Matrix E1 = R1.times(IC1);
		Matrix q1 = mock.getQuaternion(START_FRAME);

		Pose pose1 = new Pose();
		pose1.setQw(q1.get(0, 0));
		pose1.setQx(q1.get(1, 0));
		pose1.setQy(q1.get(2, 0));
		pose1.setQz(q1.get(3, 0));
		pose1.setCx(-IC1.get(0, 3));
		pose1.setCy(-IC1.get(1, 3));
		pose1.setCz(-IC1.get(2, 3));

		Matrix R2 = mock.getR(END_FRAME);
		Matrix IC2 = mock.getIC(END_FRAME);
		Matrix E2 = R2.times(IC2);
		Matrix q2 = mock.getQuaternion(END_FRAME);

		Pose pose2 = new Pose();
		pose2.setQw(q2.get(0, 0));
		pose2.setQx(q2.get(1, 0));
		pose2.setQy(q2.get(2, 0));
		pose2.setQz(q2.get(3, 0));
		pose2.setCx(-IC2.get(0, 3));
		pose2.setCy(-IC2.get(1, 3));
		pose2.setCz(-IC2.get(2, 3));

		cameras.add(pose1);
		cameras.add(pose2);

		Random rand = new Random();

		ArrayList<Matrix> initPointRes = new ArrayList<Matrix>();
		// 3D points and 2D points
		// pl("noise");
		for (int i = 0; i < mock.getWorldCoordinates().size(); i++) {
			Point3D pt3 = new Point3D();
			double xNoise = rand.nextDouble() * 1 - 0.5;
			double yNoise = rand.nextDouble() * 1 - 0.5;
			double zNoise = rand.nextDouble() * 1 - 0.5;
			// pl(xNoise);
			// pl(yNoise);
			// pl(zNoise);
			pt3.setX(mock.getWorldCoordinates().get(i).get(0, 0) + xNoise);
			pt3.setY(mock.getWorldCoordinates().get(i).get(1, 0) + yNoise);
			pt3.setZ(mock.getWorldCoordinates().get(i).get(2, 0) + zNoise);

			point3Ds.add(pt3);

			ArrayList<Point2D> pts2 = new ArrayList<Point2D>();
			Point2D pt21 = new Point2D();
			pt21.setX(mock.getKeypoints(START_FRAME).get(i).x);
			pt21.setY(mock.getKeypoints(START_FRAME).get(i).y);

			Point2D pt22 = new Point2D();
			pt22.setX(mock.getKeypoints(END_FRAME).get(i).x);
			pt22.setY(mock.getKeypoints(END_FRAME).get(i).y);

			pts2.add(pt21);
			pts2.add(pt22);
			observations.add(pts2);

			Matrix res = new Matrix(3, 1);
			res.set(0, 0, Math.pow(mock.getWorldCoordinates().get(i).get(0, 0) - point3Ds.get(i).getX(), 2));
			res.set(1, 0, Math.pow(mock.getWorldCoordinates().get(i).get(1, 0) - point3Ds.get(i).getY(), 2));
			res.set(2, 0, Math.pow(mock.getWorldCoordinates().get(i).get(2, 0) - point3Ds.get(i).getZ(), 2));
			initPointRes.add(res);
		}

		ARUtils.bundleAdjust(cameras, point3Ds, observations, 100);

		// ArrayList<Matrix> pointRes = new ArrayList<Matrix>();
		// for (int i = 0; i < mock.getWorldCoordinates().size(); i++) {
		// p(mock.getWorldCoordinates().get(i).get(0, 0) + ", " +
		// mock.getWorldCoordinates().get(i).get(1, 0) + ", "
		// + mock.getWorldCoordinates().get(i).get(2, 0) + "\t\t\t" +
		// point3Ds.get(i).getX() + ", "
		// + point3Ds.get(i).getY() + ", " + point3Ds.get(i).getZ() + "\n");
		// Matrix res = new Matrix(3, 1);
		// res.set(0, 0, Math.pow(mock.getWorldCoordinates().get(i).get(0, 0) -
		// point3Ds.get(i).getX(), 2));
		// res.set(1, 0, Math.pow(mock.getWorldCoordinates().get(i).get(1, 0) -
		// point3Ds.get(i).getY(), 2));
		// res.set(2, 0, Math.pow(mock.getWorldCoordinates().get(i).get(2, 0) -
		// point3Ds.get(i).getZ(), 2));
		// pointRes.add(res);
		// }
		//
		// pl("");
		// pl("residuals for 3D points (before)");
		// for (int i = 0; i < pointRes.size(); i++) {
		// pl(initPointRes.get(i).normF());
		// }
		//
		// pl("");
		// pl("residuals for 3D points (after)");
		// for (int i = 0; i < pointRes.size(); i++) {
		// pl(pointRes.get(i).normF());
		// }

	}

	public void boof() throws IOException {

		// Because the Bundle Adjustment in the Large data set is popular, a
		// file reader and writer is included
		// with BoofCV. BoofCV uses two data types to describe the parameters in
		// a bundle adjustment problem
		// BundleAdjustmentSceneStructure is used for camera parameters, camera
		// locations, and 3D points
		// BundleAdjustmentObservations for image observations of 3D points
		// ExampleMultiViewSceneReconstruction gives a better feel for these
		// data structures or you can look
		// at the source code of CodecBundleAdjustmentInTheLarge
		CodecBundleAdjustmentInTheLarge parser = new CodecBundleAdjustmentInTheLarge();

		parser.parse(new File("problem-16-22106-pre.txt"));

		// Print information which gives you an idea of the problem's scale
		System.out.println("Optimizing " + parser.scene.getParameterCount() + " parameters with "
				+ parser.observations.getObservationCount() + " observations\n\n");

		// Configure the sparse Levenberg-Marquardt solver
		ConfigLevenbergMarquardt configLM = new ConfigLevenbergMarquardt();
		// Important tuning parameter. Won't converge to a good solution if
		// picked improperly. Small changes
		// to this problem and speed up or slow down convergence and change the
		// final result. This is true for
		// basically all solvers.
		configLM.dampeningInitial = 1e-3;
		// Improves Jacobian matrix's condition. Recommended in general but not
		// important in this problem
		configLM.hessianScaling = true;

		ConfigBundleAdjustment configSBA = new ConfigBundleAdjustment();
		configSBA.configOptimizer = configLM;

		// Create and configure the bundle adjustment solver
		BundleAdjustment<SceneStructureMetric> bundleAdjustment = FactoryMultiView.bundleSparseMetric(configSBA);
		// prints out useful debugging information that lets you know how well
		// it's converging
		bundleAdjustment.setVerbose(System.out, 0);
		// Specifies convergence criteria
		bundleAdjustment.configure(1e-6, 1e-6, 50);

		// Scaling each variable type so that it takes on a similar numerical
		// value. This aids in optimization
		// Not important for this problem but is for others
		ScaleSceneStructure bundleScale = new ScaleSceneStructure();
		bundleScale.applyScale(parser.scene, parser.observations);
		bundleAdjustment.setParameters(parser.scene, parser.observations);

		// Runs the solver. This will take a few minutes. 7 iterations takes
		// about 3 minutes on my computer
		long startTime = System.currentTimeMillis();
		double errorBefore = bundleAdjustment.getFitScore();
		if (!bundleAdjustment.optimize(parser.scene)) {
			throw new RuntimeException("Bundle adjustment failed?!?");
		}

		// Print out how much it improved the model
		System.out.println();
		System.out.printf("Error reduced by %.1f%%\n", (100.0 * (errorBefore / bundleAdjustment.getFitScore() - 1.0)));
		System.out.println((System.currentTimeMillis() - startTime) / 1000.0);

		// Return parameters to their original scaling. Can probably skip this
		// step.
		bundleScale.undoScale(parser.scene, parser.observations);

		// Visualize the results using a point cloud viewer
		visualizeInPointCloud(parser.scene);
	}

	private static void visualizeInPointCloud(SceneStructureMetric structure) {

		List<Point3D_F64> cloudXyz = new ArrayList<>();
		Point3D_F64 world = new Point3D_F64();
		Point3D_F64 camera = new Point3D_F64();

		for (int i = 0; i < structure.points.size; i++) {
			// Get 3D location
			SceneStructureMetric.Point p = structure.points.get(i);
			p.get(world);

			// Project point into an arbitrary view
			for (int j = 0; j < p.views.size; j++) {
				int viewIdx = p.views.get(j);
				SePointOps_F64.transform(structure.views.data[viewIdx].worldToView, world, camera);
				cloudXyz.add(world.copy());
				break;
			}
		}

		PointCloudViewer viewer = VisualizeData.createPointCloudViewer();
		viewer.setFog(true);
		viewer.setColorizer(new SingleAxisRgb.Z().fperiod(20)); // makes it
																// easier to see
																// points
																// without RGB
																// color
		viewer.setClipDistance(70); // done for visualization to make it easier
									// to separate objects with the fog
		viewer.setDotSize(1);
		viewer.setTranslationStep(0.05);
		viewer.addCloud(cloudXyz);
		viewer.setCameraHFov(UtilAngle.radian(60));

		// Give it a good initial pose. This was determined through trial and
		// error
		Se3_F64 cameraToWorld = new Se3_F64();
		cameraToWorld.T.set(-10.848385, -6.957626, 2.9747992);
		ConvertRotation3D_F64.eulerToMatrix(EulerType.XYZ, -2.734419, -0.27446, -0.24310, cameraToWorld.R);
		viewer.setCameraToWorld(cameraToWorld);

		SwingUtilities.invokeLater(() -> {
			viewer.getComponent().setPreferredSize(new Dimension(600, 600));
			ShowImages.showWindow(viewer.getComponent(), "Refined Scene", true);
		});
	}

	public static void p(Object s) {
		System.out.print(s);
	}

	public static void pl(Object s) {
		System.out.println(s);
	}
}