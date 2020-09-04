import org.opencv.core.Core;

import ARPipeline.ARPipeline;
import ARPipeline.CameraIntrinsics;
import ARPipeline.MockPipeline;
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
		// arBootstrapper.start();
		arBootstrapper.tests();
	}

	public void tests() {
		Matrix K = Matrix.identity(4, 4);
		K.set(0, 0, CameraIntrinsics.fx);
		K.set(0, 1, CameraIntrinsics.s);
		K.set(0, 2, CameraIntrinsics.cx);
		K.set(1, 0, 0.0);
		K.set(1, 1, CameraIntrinsics.fy);
		K.set(1, 2, CameraIntrinsics.cy);
		K.set(2, 0, 0.0);
		K.set(2, 1, 0.0);
		K.set(2, 2, 1.0);

		Matrix KInv = K.inverse();

		Matrix worldPoint = new Matrix(4, 1);
		worldPoint.set(0, 0, -1000);
		worldPoint.set(1, 0, 1000);
		worldPoint.set(2, 0, 1000);
		worldPoint.set(3, 0, 1);

		Matrix IC = Matrix.identity(4, 4);
		IC.set(0, 3, 1000);
		IC.set(1, 3, 0);
		IC.set(2, 3, 0);

		Matrix R = Matrix.identity(4, 4);
		R.set(1, 1, Math.cos(Math.PI / 4));
		R.set(2, 2, Math.cos(Math.PI / 4));
		R.set(1, 2, -Math.sin(Math.PI / 4));
		R.set(2, 1, Math.sin(Math.PI / 4));

		Matrix E = R.times(IC);

		Matrix goal = new Matrix(3, 1);
		goal.set(0, 0, worldPoint.get(0, 0) + IC.get(0, 3));
		goal.set(1, 0, worldPoint.get(1, 0) + IC.get(1, 3));
		goal.set(2, 0, worldPoint.get(2, 0) + IC.get(2, 3));
		goal = goal.times(1 / goal.normF());
		System.out.println("goal");
		goal.print(5, 4);

		Matrix EInv = E.inverse();

		Matrix x = K.times(E).times(worldPoint);
		x = x.times(1 / x.get(2, 0));
		x.print(5, 20);

		x.set(3, 0, 1);

		Matrix partial = KInv.times(x);
		partial.print(5, 20);
		Matrix unprojected = EInv.times(KInv).times(x);
		unprojected.set(0, 0, 1 / unprojected.get(0, 0));
		unprojected.set(1, 0, 1 / unprojected.get(1, 0));
		unprojected.set(2, 0, 1 / unprojected.get(2, 0));
		unprojected = unprojected.times(1 / unprojected.getMatrix(0, 2, 0, 0).normF());
		unprojected.print(5, 20);

	}
}