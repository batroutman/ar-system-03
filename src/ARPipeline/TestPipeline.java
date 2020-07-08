package ARPipeline;

import org.opencv.features2d.ORB;
import org.opencv.core.MatOfKeyPoint;

import java.util.ArrayList;

import org.opencv.core.Mat;

public class TestPipeline extends ARPipeline{
	
	float rotAngle = 0;
	
	protected Thread mainThread = new Thread() {
		@Override
		public void run() {
			mainloop();
		}
	};

	public TestPipeline(){
		super();
	}

	public TestPipeline(FrameBuffer inputFrameBuffer) {
		super();
		this.setInputFrameBuffer(inputFrameBuffer);
	}

	public TestPipeline(FrameBuffer inputFrameBuffer, PoseBuffer outputPoseBuffer) {
		super();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputPoseBuffer(outputPoseBuffer);
	}
	
	public TestPipeline(FrameBuffer inputFrameBuffer, FrameBuffer outputFrameBuffer) {
		super();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputFrameBuffer(outputFrameBuffer);
	}

	public TestPipeline(FrameBuffer inputFrameBuffer, PoseBuffer outputPoseBuffer, FrameBuffer outputFrameBuffer) {
		super();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputPoseBuffer(outputPoseBuffer);
		this.setOutputFrameBuffer(outputFrameBuffer);
	}

	public void start() {
		this.mainThread.start();
	}
	public void stop() {
		this.mainThread.interrupt();
	}
	
	Mat oldDesc = new Mat();
	
	protected void mainloop() {
		Frame currentFrame = this.inputFrameBuffer.getCurrentFrame();
		boolean keepGoing = true;
		while (keepGoing) {
			
			if (currentFrame == null) {
				keepGoing = false;
				
				getStats(oldDesc);
				
				continue;
			}
			
			// find ORB features
			int patchSize = 5;
			ORB orb = ORB.create(100);
			orb.setScoreType(ORB.FAST_SCORE);
			orb.setPatchSize(patchSize);
			orb.setNLevels(1);
			MatOfKeyPoint keypoints = new MatOfKeyPoint();
			Mat descriptors = new Mat();
			Mat image = ARUtils.frameToMat(currentFrame);
			orb.detectAndCompute(image, new Mat(), keypoints, descriptors);
			ARUtils.boxHighlight(currentFrame, keypoints, patchSize);
			oldDesc = descriptors;
			
			// rotate cube as demo that pose can be modified and displayed
			rotAngle += 0.002f;
			Pose pose = new Pose();
			pose.setR11((float)Math.cos(rotAngle));
			pose.setR22((float)Math.cos(rotAngle));
			pose.setR12(-(float)Math.sin(rotAngle));
			pose.setR21((float)Math.sin(rotAngle));
			
			synchronized (this.outputPoseBuffer) {
				this.outputPoseBuffer.pushPose(pose);
			}
			
			// for demo, just push the unaltered frame along to the output buffer
			synchronized (this.outputFrameBuffer) {
				this.outputFrameBuffer.pushFrame(currentFrame);
			}

			currentFrame = this.inputFrameBuffer.getCurrentFrame();
			
			
			
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
	
	

}