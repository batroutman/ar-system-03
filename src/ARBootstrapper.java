import org.opencv.core.Core;

import ARPipeline.ARPipeline;
import ARPipeline.CameraIntrinsics;
import ARPipeline.MockPipeline;
import ARPipeline.MockPointData;
import ARPipeline.SingletonFrameBuffer;
import ARPipeline.SingletonPoseBuffer;
import Jama.Matrix;

public class ARBootstrapper {

	String SAMPLE_PATH = "src/samples/";
	String filename = SAMPLE_PATH + "roomFloor01_270.avi";

	public ARBootstrapper() {

	}

	public void start() {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

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
		arBootstrapper.start();
		// arBootstrapper.tests();
	}

	public void tests() {
		Matrix K = new Matrix(3, 3);
		K.set(0, 0, CameraIntrinsics.fx);
		K.set(0, 1, CameraIntrinsics.s);
		K.set(0, 2, CameraIntrinsics.cx);
		K.set(1, 0, 0.0);
		K.set(1, 1, CameraIntrinsics.fy);
		K.set(1, 2, CameraIntrinsics.cy);
		K.set(2, 0, 0.0);
		K.set(2, 1, 0.0);
		K.set(2, 2, 1.0);

		Matrix E = Matrix.identity(3, 4);
		Matrix R = Matrix.identity(3, 3);
		Matrix C = Matrix.identity(3, 4);

		double rot = Math.PI / 2;

		// R.set(1, 1, Math.cos(rot));
		// R.set(2, 2, Math.cos(rot));
		// R.set(1, 2, -Math.sin(rot));
		// R.set(2, 1, Math.sin(rot));

		C.set(2, 3, -500);
		System.out.println("C:");
		C.print(5, 4);

		Matrix cameraMatrix = K.times(R).times(C);
		System.out.println("cameraMatrix:");
		cameraMatrix.print(5, 4);

		Matrix point = new Matrix(4, 1);
		point.set(0, 0, -200);
		point.set(1, 0, 0);
		point.set(2, 0, 1000);
		point.set(3, 0, 1);

		System.out.println("point (3D):");
		point.print(5, 4);
		Matrix projected = cameraMatrix.times(point);

		System.out.println("Un-normalized projected point:");
		projected.print(5, 4);
		projected = projected.times(1 / projected.get(2, 0));

		System.out.println("Normalized projected point:");
		projected.print(5, 4);

		MockPointData mock = new MockPointData();
		mock.getWorldCoordinates().get(2).print(5, 4);
		for (int i = 0; i < 10; i++) {
			System.out.println(mock.getKeypoints(i).get(2).x + ", " + mock.getKeypoints(i).get(2).y);
		}

	}
}