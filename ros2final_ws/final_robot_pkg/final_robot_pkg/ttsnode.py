#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import tempfile
import os

class TextToSpeechNode(Node):
    def __init__(self):
        super().__init__('text_to_speech_node')

        # USB SpeakerPhone (mono device)
        self.audio_device = "plughw:1,0"

        # espeak-ng parameters (your requested settings)
        self.voice = "en-us"
        self.speed = "120"      # -s
        self.pitch = "40"       # -p
        self.volume = "800"     # -a  !!!! VERY LOUD !!!!
        self.rate = "16000"     # for aplay

        self.subscription = self.create_subscription(
            String,
            'tts_text',
            self.on_tts,
            10
        )

        self.get_logger().info("TTS Node ready. Listening on /tts_text")

    def on_tts(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"TTS request: {text}")

        # Temp file for WAV output
        with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as tmp:
            wav_path = tmp.name

        try:
            # Generate WAV using espeak-ng
            espeak_cmd = [
                "espeak-ng",
                "-v", self.voice,
                "-s", self.speed,
                "-p", self.pitch,
                "-a", self.volume,
                "--stdout",
                text
            ]

            self.get_logger().info(f"Running espeak-ng...")
            with open(wav_path, "wb") as f:
                subprocess.run(espeak_cmd, stdout=f, check=True)

            # Playback using aplay
            aplay_cmd = [
                "aplay",
                "-D", self.audio_device,
                "--channels=1",
                "--rate", self.rate,
                wav_path
            ]

            self.get_logger().info("Playing audio...")
            subprocess.run(aplay_cmd)

        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")

        finally:
            if os.path.exists(wav_path):
                os.remove(wav_path)


def main(args=None):
    rclpy.init(args=args)
    node = TextToSpeechNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

