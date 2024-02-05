# Human-robot interaction with LLMs
Currently, the perception of robots is often within an industrial scope and rarely considered in terms of interaction between humans and robots. However, over the last few years, it is possible to observe that this communication, as demonstrated by assistants like Alexa or even with communication with ChatGPT, has grown significantly. There is a growing interest among the population in this natural communication between machines and humans.

## How to start
Set up your .env file
```sh
OPENAI_API_KEY='<YOUR API KEY HERE>'
```

You can start the application with the usual ros2 commands, for that you will need ros2 installed in your computer, instead you can use the dockerfile offered in the repository.
```sh
source /opt/ros/<distro>/setup.bash
colcon build
source install/setup.bash
```

You can try using Docker \
To build the dockerfile (remember to be in the folder with the Dockerfile)
```sh
docker build -t simple_hri:1.0 .
```

Before starting the dockerfile you will need a Riva server, you can do that by downloading from NVIDIA NGC

Start the dockerfile

```sh
docker run -it --rm --privileged -v ${PWD}/src:/QA_ws/src simple_hri:1.0
```

To begin the application \
Set the environment
```sh
colcon build
source install/setup.bash
```

You can know start the servers by
```sh
ros2 launch simple_qa question_answer.py
```

In other terminal you can start the question and answer pipeline
```sh
ros2 run simple_qa qa
```
