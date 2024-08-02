from ffmpeg.ffmpeg import FFmpeg
from ffmpeg.progress import Progress

def main():

    # Same as command: ffmpeg -f v4l2 -i /dev/video0 -f v4l2 /dev/video2
    ffmpeg = ffmpeg = (
        FFmpeg()
        .option("y")
        .input(
            '/dev/video0',
            f='v4l2',
        )
        .output(
            '/dev/video10', 
            f='v4l2',
        )
    )

    @ffmpeg.on("progress")
    def time_to_terminate(progress: Progress):
        print(f"Progress: {get_progress_as_string(progress)}")

    try:
        result = ffmpeg.execute()
    except KeyboardInterrupt as e:
        ffmpeg.terminate()
    except Exception as e:
        exit


def get_progress_as_string(progress: Progress):
    return f"Progress - frame: {progress.frame}, fps: {progress.fps}, size: {progress.size}, " + \
            f"time: {progress.time}, bitrate: {progress.bitrate}, speed: {progress.speed}"

if __name__ == '__main__':
    main()