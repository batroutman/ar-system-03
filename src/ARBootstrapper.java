import org.opencv.core.Core;

import ARPipeline.ARPipeline;
import ARPipeline.CameraIntrinsics;
import ARPipeline.Correspondence2D2D;
import ARPipeline.MockPipeline;
import ARPipeline.MockPointData;
import ARPipeline.SingletonFrameBuffer;
import ARPipeline.SingletonPoseBuffer;
import Jama.Matrix;
import Jama.SingularValueDecomposition;

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

		// Matrix KInv = K.inverse();
		//
		// Matrix worldPoint = new Matrix(4, 1);
		// worldPoint.set(0, 0, -1000);
		// worldPoint.set(1, 0, 1000);
		// worldPoint.set(2, 0, 1000);
		// worldPoint.set(3, 0, 1);
		//
		// Matrix IC = Matrix.identity(4, 4);
		// IC.set(0, 3, 100);
		// IC.set(1, 3, 0);
		// IC.set(2, 3, 0);
		//
		// Matrix goal = new Matrix(3, 1);
		// goal.set(0, 0, worldPoint.get(0, 0) + IC.get(0, 3));
		// goal.set(1, 0, worldPoint.get(1, 0) + IC.get(1, 3));
		// goal.set(2, 0, worldPoint.get(2, 0) + IC.get(2, 3));
		// goal = goal.times(1 / goal.normF());
		// System.out.println("goal: ");
		// goal.print(5, 10);
		//
		// Matrix R = Matrix.identity(4, 4);
		// R.set(1, 1, Math.cos(Math.PI / 4));
		// R.set(2, 2, Math.cos(Math.PI / 4));
		// R.set(1, 2, -Math.sin(Math.PI / 4));
		// R.set(2, 1, Math.sin(Math.PI / 4));
		//
		// Matrix E = R.times(IC);
		// Matrix EInv = E.inverse();

		int END_FRAME = 99;

		// init mock
		MockPointData mock = new MockPointData();

		// get camera pose for frame 0
		Matrix R1 = mock.getR(0);
		Matrix IC1 = mock.getIC(0);
		Matrix E1 = R1.times(IC1);

		// get camera pose for frame 100
		Matrix R2 = mock.getR(END_FRAME);
		Matrix IC2 = mock.getIC(END_FRAME);
		Matrix E2 = R2.times(IC2);
		IC2.print(10, 5);
		E2.print(10, 5);
		System.out.println("target scale: " + E2.getMatrix(0, 2, 3, 3).normF());

		// get first correspondence for frame 0 and 100
		Correspondence2D2D c = new Correspondence2D2D();
		c.setU1(mock.getKeypoints(0).get(0).x);
		c.setV1(mock.getKeypoints(0).get(0).y);
		c.setU2(mock.getKeypoints(100).get(0).x);
		c.setV2(mock.getKeypoints(100).get(0).y);

		// normalize translation vector on frame 100 and pretend this is an
		// essential decomposition
		Matrix t2 = E2.getMatrix(0, 2, 3, 3);
		t2 = t2.times(1 / t2.normF());
		E2.set(0, 3, t2.get(0, 0));
		E2.set(1, 3, t2.get(1, 0));
		E2.set(2, 3, t2.get(2, 0));
		E2.print(10, 5);

		// init old_score
		double oldScore = 0;

		// init scale = 1
		double scale = 581.4722693;

		// init scale_jump = 10000
		double scaleJump = 10000;

		// init go_right = true
		boolean goRight = true;

		// init first_pass = true
		boolean firstPass = true;

		boolean keepGoing = true;
		// while keepgoing:
		while (keepGoing) {
			// // generate fundamental matrix between poses with some scale
			// // // - calculate C =
			// R1Inv.times(E1.getMatrix(0,2,3,3)).times(-1)
			Matrix R1Inv = R1.getMatrix(0, 2, 0, 2).inverse();
			Matrix C = R1Inv.times(E1.getMatrix(0, 2, 3, 3)).times(-1);
			Matrix Ch = new Matrix(4, 1);
			Ch.set(0, 0, C.get(0, 0));
			Ch.set(1, 0, C.get(1, 0));
			Ch.set(2, 0, C.get(2, 0));
			Ch.set(3, 0, 1);

			// // // - E2 = [R2 | scale * t]
			Matrix E2scaled = E2.copy();
			E2scaled.set(0, 3, E2.get(0, 3) * scale);
			E2scaled.set(1, 3, E2.get(1, 3) * scale);
			E2scaled.set(2, 3, E2.get(2, 3) * scale);
			System.out.println("E2scaled");
			E2scaled.print(15, 5);

			// // // - calculate e' = E2.times(C)
			Matrix Pprime = K.getMatrix(0, 2, 0, 2).times(E2scaled.getMatrix(0, 2, 0, 3));
			Matrix e = Pprime.times(Ch);
			System.out.println("e':");
			e.print(15, 5);
			K.getMatrix(0, 2, 0, 2).times(E1.getMatrix(0, 2, 0, 3)).times(Ch).print(15, 5);

			// // // - generate [e']x
			Matrix ex = new Matrix(3, 3);
			ex.set(0, 1, -e.get(2, 0));
			ex.set(0, 2, e.get(1, 0));
			ex.set(1, 0, e.get(2, 0));
			ex.set(1, 2, -e.get(0, 0));
			ex.set(2, 0, -e.get(1, 0));
			ex.set(2, 1, e.get(0, 0));

			System.out.println("ex:");
			ex.print(15, 5);

			// // // - calculate P+ = E1.inverse()
			K.times(E1).print(15, 5);
			K.times(E1).inverse().print(15, 5);
			Matrix P = K.getMatrix(0, 2, 0, 2).times(E1.getMatrix(0, 2, 0, 3));
			SingularValueDecomposition svd = P.transpose().svd();
			Matrix Splus = new Matrix(3, 3);
			Splus.set(0, 0, 1 / svd.getS().get(0, 0));
			Splus.set(1, 1, 1 / svd.getS().get(1, 1));
			Splus.set(2, 2, 1 / svd.getS().get(2, 2));
			Matrix U = svd.getV();
			Matrix V = svd.getU();
			Matrix PInv = V.times(Splus).times(U.transpose());

			// // // - F = [e']x(E2.times(E1.inverse()))
			Matrix F = ex.times(Pprime.times(PInv));
			p("F");
			F.print(15, 5);
			F = F.times(1 / F.get(2, 2));
			p("F after homogeneousaztion");
			F.print(15, 5);

			// test a correspondence
			Matrix x = new Matrix(3, 1);
			x.set(0, 0, c.getU1());
			x.set(1, 0, c.getV1());
			x.set(2, 0, 1);

			Matrix xprime = new Matrix(3, 1);
			xprime.set(0, 0, c.getU2());
			xprime.set(1, 0, c.getV2());
			xprime.set(2, 0, 1);

			p("x'Fx");
			xprime.transpose().times(F).times(x).print(15, 5);
			x.set(0, 0, x.get(0, 0) + 50);
			p("x'Fx (after x moved out of place)");
			xprime.transpose().times(F).times(x).print(15, 5);

			// // score = | x2 * F * x1 |
			// // if score < THRESHOLD:
			// // // keepgoing = false
			// // // continue
			// // if first_pass:
			// // // first_pass = false
			// // // old_score = score
			// // // continue
			// // if score > old_score:
			// // // go_right = !go_right
			// // // scale_jump = scale_jump / 2
			// // scale = go_right ? scale + scale_jump : scale - scale_jump
			keepGoing = false;
		}

	}

	public static void p(String s) {
		System.out.print(s);
	}

	public static void pl(String s) {
		System.out.println(s);
	}
}