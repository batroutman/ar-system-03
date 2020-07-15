package ARPipeline;

import org.opencv.features2d.ORB;
import org.opencv.core.MatOfKeyPoint;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;

public class TestPipeline extends ARPipeline{
	
	double TOLERANCE = 100.0;
	double SEARCH_RADIUS = 20.0;
	
	ArrayList<KeyFrame> keyframes = new ArrayList<KeyFrame>();
	KeyFrame currentKeyFrame = null;
	
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
	
	public static KeyFrame generateKeyFrame(Mat descriptors, MatOfKeyPoint keypoints) {
		
		KeyFrame keyframe = new KeyFrame();
		Pose pose = new Pose();
		pose.setOrigin();
		keyframe.setPose(pose);
		
		List<KeyPoint> keypointList = keypoints.toList();
		MapPoint firstMp = new MapPoint(descriptors.row(0));
		// may have issues here. negative y? offset?
		firstMp.setUV(keypointList.get(0).pt.x, keypointList.get(0).pt.y);
		MapPointKDTree tree = new MapPointKDTree(firstMp);
		
		for (int i = 1; i < keypointList.size(); i++) {
			MapPoint mp = new MapPoint(descriptors.row(i));
			mp.setUV(keypointList.get(i).pt.x, keypointList.get(i).pt.y);
			tree.insert(mp);
		}
		
		System.out.println("depth: " + tree.depth());
		
		keyframe.setDescriptors(tree);
		return keyframe;
		
	}
	
	public ArrayList<Correspondence> getCorrespondences(KeyFrame keyframe, Mat descriptors, MatOfKeyPoint keypoints, double tolerance) {
		
		ArrayList<Correspondence> correspondences = new ArrayList<Correspondence>();
		List<KeyPoint> keypointList = keypoints.toList();
		
		for (int rowIndex = 0; rowIndex < descriptors.rows(); rowIndex++) {
			Mat row = descriptors.row(rowIndex);
			MapPoint nearest = keyframe.getDescriptors().nearest(row);
			if (ARUtils.descriptorDistance(row, nearest.getDescriptor()) <= tolerance) {
				Correspondence c = new Correspondence();
				c.setDescriptor1(nearest.getDescriptor());
				c.setU1(nearest.getU());
				c.setV1(nearest.getV());
				c.setDescriptor2(row);
				c.setU2(keypointList.get(rowIndex).pt.x);
				c.setV2(keypointList.get(rowIndex).pt.y);
				
				correspondences.add(c);
			}
		}
		
		return correspondences;
		
	}
	
	Mat oldDesc = new Mat();
	int count  = 0;
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
			
			// what do I want to do?
			// find correspondences between descriptors and mappoints in used in current keyframe
			//		a. if I have 0 keyframes, generate one and set pose to origin. continue.
					if (this.keyframes.size() == 0) {
						this.currentKeyFrame = generateKeyFrame(descriptors, keypoints);
						this.keyframes.add(this.currentKeyFrame);
						
						System.out.println(this.keyframes.size());
					}
			//		b. otherwise, 
					else {
						// calculate correspondences
						ArrayList<Correspondence> correspondences = this.getCorrespondences(this.currentKeyFrame, descriptors, keypoints, TOLERANCE);
						System.out.println("Num of Correspondences: " + correspondences.size());
						byte [] red = { (byte)255, 0, 0 };
						for(Correspondence c:correspondences) {
							ARUtils.boxHighlight(currentFrame, (int)c.getU2(), (int)c.getV2(), red, 8);
						}
						
			// 			if I have at least 40(?) correspondences, then simply compute 5-point sfm with them. continue
			//			i. else (I don't have at least 40(?) correspondences), but I have at least 10, then generate new keyframe and use the same correspondences for sfm. set current keyframe to new keyframe. continue.
			//			ii. else (<10 correspondences), check other keyframes for keyframe with most correspondences. If >= 10 correspondences, set as current keyframe and use the correspondences for sfm. if < 40, generate new keyframe.
			// 			iii. else, tracking lost (handle this later)
					}	

			
					count++;
			
			
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