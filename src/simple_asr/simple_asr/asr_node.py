import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import io
import os
import sys
import time
import warnings
import wave
from pathlib import Path
from typing import Callable, Dict, Generator, Iterable, List, Optional, TextIO, Union

from grpc._channel import _MultiThreadedRendezvous
import sounddevice
import riva

import riva.client.proto.riva_asr_pb2 as rasr
import riva.client.proto.riva_asr_pb2_grpc as rasr_srv
from riva.client.auth import Auth


from simple_asr.audio_io import MicrophoneStream


class AsrNode(Node):

    def __init__(self):
        super().__init__('asr_node')

        self.publisher_ = self.create_publisher(String, 'asr_output', 10)
        #Check the impact of this timer period parameter.
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.asr_callback)
        self.msg = String()


        try :
            self.input_device = int(sys.argv[1])
        except:
            input_device_specs = sounddevice.query_devices(device='logitech')
            self.input_device = int(input_device_specs['index'])
            self.get_logger().info(f"Input audio device setted as  > '{input_device_specs['name']}'")

        self.auth = riva.client.Auth()
        self.asr_service = riva.client.ASRService(self.auth)
        self.config = riva.client.StreamingRecognitionConfig(
        config=riva.client.RecognitionConfig(
            encoding=riva.client.AudioEncoding.LINEAR_PCM,
            language_code='en-US',
            max_alternatives=1,
            profanity_filter=False,
            enable_automatic_punctuation=True,
            verbatim_transcripts=True,
            sample_rate_hertz=16000,
            audio_channel_count=1,
        ),
        interim_results=True,
        )
        # Check the parametrization of the scalar in this word boosting
        riva.client.add_word_boosting_to_config(self.config, ['miss piggy'], 1000.0)
    
    def asr_callback(self):
        with MicrophoneStream(
        16000,
        1600,
            device=self.input_device,
        ) as audio_chunk_iterator:
            self.print_streaming(
                responses=self.asr_service.streaming_response_generator(
                    audio_chunks=audio_chunk_iterator,
                    streaming_config=self.config,
                ),
                ## Update output_file to a better location than inside the workspace (the way it is rn)
                output_file="./asr_log.txt",
                show_intermediate=True,
            )
        return response

    def print_streaming(
        self,
        responses: Iterable[rasr.StreamingRecognizeResponse],
        output_file: Optional[Union[Union[os.PathLike, str, TextIO], List[Union[os.PathLike, str, TextIO]]]] = None,
        additional_info: str = 'no',
        word_time_offsets: bool = False,
        show_intermediate: bool = False,
        file_mode: str = 'w',
    ) -> None:
        if output_file is None:
            output_file = [sys.stdout]
        elif not isinstance(output_file, list):
            output_file = [output_file]
        file_opened = [False] * len(output_file)
        try:
            for i, elem in enumerate(output_file):
                if isinstance(elem, io.TextIOBase):
                    file_opened[i] = False
                else:
                    file_opened[i] = True
                    output_file[i] = Path(elem).expanduser().open(file_mode)
            start_time = time.time()  # used in 'time` additional_info
            num_chars_printed = 0  # used in 'no' additional_info
            for response in responses:
                if not response.results:
                    continue
                partial_transcript = ""
                for result in response.results:
                    if not result.alternatives:
                        continue
                    transcript = result.alternatives[0].transcript
                    if additional_info == 'no':
                        if result.is_final:
                            for i, alternative in enumerate(result.alternatives):
                                for f in output_file:
                                    self.msg.data = str((f'(alternative {i + 1})' if i > 0 else '') + f' {alternative.transcript}')
                                    self.publisher_.publish(self.msg)
                                    f.write(
                                        f'##'
                                        + (f'(alternative {i + 1})' if i > 0 else '')
                                        + f' {alternative.transcript}\n'
                                    )
                        else:
                            partial_transcript += transcript
        finally:
            for fo, elem in zip(file_opened, output_file):
                if fo:
                    elem.close()




def main(args=None):
    rclpy.init(args=args)

    asr_node = AsrNode()

    rclpy.spin(asr_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()