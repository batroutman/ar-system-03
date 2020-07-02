package ARPipeline;

public class SingletonFrameBuffer implements FrameBuffer{

	Frame currentFrame;
	
	public SingletonFrameBuffer() {
		
	}
	
	@Override
	public void pushFrame(Frame frame) {
		this.currentFrame = frame;
	}

	@Override
	public Frame getCurrentFrame() {
		return this.currentFrame;
	}

}
