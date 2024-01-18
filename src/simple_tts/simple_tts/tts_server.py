#!/usr/bin/python3
from simple_interfaces.srv import TTS

import rclpy
from rclpy.node import Node
import riva.client
import sys
import argparse
import time
import wave
from pathlib import Path
import platform
import riva.client
from riva.client.argparse_utils import add_connection_argparse_parameters
import pyaudio
import sounddevice

import queue
from typing import Dict, Union, Optional

class SoundCallBack:
    def __init__(
        self, output_device_index: Optional[int], sampwidth: int, nchannels: int, framerate: int,
    ) -> None:
        self.pa = pyaudio.PyAudio()
        self.stream = self.pa.open(
            output_device_index=output_device_index,
            format=self.pa.get_format_from_width(sampwidth),
            channels=nchannels,
            rate=framerate,
            output=True,
        )
        self.opened = True

    def __call__(self, audio_data: bytes, audio_length: float = None) -> None:
        self.stream.write(audio_data)

    def __enter__(self):
        return self

    def __exit__(self, type_, value, traceback) -> None:
        self.close()

    def close(self) -> None:
        self.stream.close()
        self.pa.terminate()
        self.opened = False


class TtsService(Node):

    def __init__(self):
        super().__init__('simple_tts')
        # audio_io.list_output_devices()
        self.srv = self.create_service(TTS, 'text_to_speech', self.tts_callback)
        output_device_spec = sounddevice.query_devices(device='hyperx')
        self.output_device = int(output_device_spec['index'])
        self.get_logger().info(f"Output audio device setted as  > '{output_device_spec['name']}'")

    def tts_callback(self, request, response):
        auth = riva.client.Auth()
        service = riva.client.SpeechSynthesisService(auth)
        nchannels = 1
        sampwidth = 2
        sample_rate_hz = 16000
        sound_stream, out_f = None, None
        try:
            sound_stream = SoundCallBack(
                self.output_device, nchannels=nchannels, sampwidth=sampwidth, framerate=sample_rate_hz
            )
            if self.output_device is not None:
                out_f = wave.open(str(self.output_device), 'wb')
                out_f.setnchannels(nchannels)
                out_f.setsampwidth(sampwidth)
                out_f.setframerate(sample_rate_hz)

            text = request.text
            self.get_logger().info(f"Generating audio for request...  > '{text}'")
            start = time.time()
            responses = service.synthesize_online(
                text, None, 'en-US', sample_rate_hz=sample_rate_hz
            )
            first = True
            for resp in responses:
                stop = time.time()
                if first:
                    print(f"Time to first audio: {(stop - start):.3f}s")
                    first = False
                if sound_stream is not None:
                    sound_stream(resp.audio)
                if out_f is not None:
                    out_f.writeframesraw(resp.audio)
        finally:
            if out_f is not None:
                out_f.close()
            if sound_stream is not None:
                sound_stream.close()

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = TtsService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()