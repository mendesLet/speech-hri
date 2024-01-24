# Uso de processamento de linguagem natural para interação entre humano e robô
Atualmente a visão sobre robôs é sempre um âmbito mais industrial e raramente pensado na interação com comunicação entre humanos e robôs, mas ao decorrer dos últimos anos é possível perceber que essa comunicação, como mostrado por assistentes como a alexa, ou mesmo com a comunicação com o chatGPT, tem crescido muito e é de interesse da população essa comunicação natural entre máquina e homem.

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
## Artigo
[The use of Normal language processing in Human-robot interaction](https://docs.google.com/document/d/1ervE7q6R8rKWfHUToJU5DKjqzcGiPN-L9vvx0U6cPo0/edit?usp=sharing)
