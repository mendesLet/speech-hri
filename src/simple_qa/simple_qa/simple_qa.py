from simple_tts.tts_client import TtsClient
import simple_asr
from simple_ww.wakeword_node import run_ww, WakeWordNode
import simple_interfaces
from simple_nlu.nlu_client import NLUClient
import click
import logging

from time import sleep


import rclpy

def main():
    rclpy.init()
    tts = TtsClient()
    nlu = NLUClient()
    ww = WakeWordNode('stop')

    print(f"Starting Question Answer")

    while not ww.word_found:
        if '?' in ww.msg:
            answer = (nlu.send_request(ww.msg)).answer
            tts.send_request(answer)
            ww.msg = ""
            pass
        rclpy.spin_once(ww)        

if __name__ == '__main__':
    main()