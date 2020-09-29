import java.util.ArrayList;
import java.util.Random;

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
		arBootstrapper.tests();
	}

	public void tests() {

		MockPointData mock = new MockPointData();
		ArrayList<Pose> cameras = new ArrayList<Pose>();
		ArrayList<Point3D> point3Ds = new ArrayList<Point3D>();
		ArrayList<ArrayList<Point2D>> observations = new ArrayList<ArrayList<Point2D>>();

		int START_FRAME = 0;
		int END_FRAME = 1;

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
		pose1.setCx(IC1.get(0, 3));
		pose1.setCy(IC1.get(1, 3));
		pose1.setCz(IC1.get(2, 3));

		Matrix R2 = mock.getR(END_FRAME);
		Matrix IC2 = mock.getIC(END_FRAME);
		Matrix E2 = R2.times(IC2);
		Matrix q2 = mock.getQuaternion(END_FRAME);

		Pose pose2 = new Pose();
		pose2.setQw(q2.get(0, 0));
		pose2.setQx(q2.get(1, 0));
		pose2.setQy(q2.get(2, 0));
		pose2.setQz(q2.get(3, 0));
		pose2.setCx(IC2.get(0, 3));
		pose2.setCy(IC2.get(1, 3));
		pose2.setCz(IC2.get(2, 3));

		cameras.add(pose1);
		cameras.add(pose2);

		pl("q1");
		q1.print(15, 10);

		pl("rot for q1");
		pose1.getRotationMatrix().print(15, 5);

		pl("q2");
		q2.print(15, 10);

		pl("rot for q2");
		pose2.getRotationMatrix().print(15, 5);

		pl("rot1 rotated by rot2");
		ARUtils.quatRot(q1, q2).print(15, 5);

		Random rand = new Random();

		ArrayList<Matrix> initPointRes = new ArrayList<Matrix>();
		// 3D points and 2D points
		for (int i = 0; i < mock.getWorldCoordinates().size(); i++) {
			Point3D pt3 = new Point3D();
			pt3.setX(mock.getWorldCoordinates().get(i).get(0, 0) + rand.nextDouble() * 20 - 10);
			pt3.setY(mock.getWorldCoordinates().get(i).get(1, 0) + rand.nextDouble() * 20 - 10);
			pt3.setZ(mock.getWorldCoordinates().get(i).get(2, 0) + rand.nextDouble() * 20 - 10);

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

		// ARUtils.bundleAdjust(cameras, point3Ds, observations, 100);

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

	public static void p(Object s) {
		System.out.print(s);
	}

	public static void pl(Object s) {
		System.out.println(s);
	}
}