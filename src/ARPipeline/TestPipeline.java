package ARPipeline;

public class TestPipeline extends ARPipeline{
	
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
	
	protected void mainloop() {
		Frame currentFrame = this.inputFrameBuffer.getCurrentFrame();
		while (currentFrame != null) {
			this.outputFrameBuffer.pushFrame(currentFrame);
			
			currentFrame = this.inputFrameBuffer.getCurrentFrame();
			
		}
	}
	
	

}