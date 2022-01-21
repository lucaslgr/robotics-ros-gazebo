<h1 align="center" ><b style="color: #03A9F5;">PROJETO 1 - TURTLEBOT3 (BURGER)</b> </h1>

<h3> :loudspeaker: O que é o projeto? </h3>
<p>O projeto tem como objetivo utilizar os respectivos pacotes para que o robo possa mapear um ambiente criado por nós e posteriormente navegar de forma autônoma trançando a melhor trajetória e se desviando de obstáculos. Este é um projeto da disciplina de <b>robótica aplicada</b> do CEFET-MG campus Leopoldina ministrada pelo professor Vinícius Schettino.</p>

<h4>Instruções fornecidas pelo professor: </h4>

- Criar um mundo no Gazebo representando sua casa
  - http://gazebosim.org/tutorials?tut=build_world
  - https://gazebosim.org/tutorials?tut=building_editor
- Criar o mapa desse mundo usando teleoperação com o Turtlebot e o pacote hector_mapping
  - http://wiki.ros.org/hector_mapping
- Implementar localização e navegação autônoma
  - Garantir que o robô consegue passar por portas
  - Garantir que o robô consegue passar por obstáculos não mapeados

<hr>

<h3>:handshake: Equipe : </h3>
<table>
    <tr>
        <th>Elvis Martins</th>
        <th>José Marcio</th>
        <th>Karine Cunha</th>
        <th>Lucas Guimarães</th>
    </tr>
</table>

<hr>

<h3 id="sumario"> 📑 Sumário </h3>

- <a href="#requisitos"> Requisitos para rodar o projeto.</a>
- <a href="#como-rodar">Como rodar o projeto.</a>
  - <a href="#como-rodar-robo-mundo">Rodar o mundo e inserir o robo dentro dele.</a>
  - <a href="#como-rodar-mapeamento">Mapeamento (hectormapping).</a>
  - <a href="#como-rodar-localizacao">Localização (amcl).</a>
  - <a href="#como-rodar-navegacao-autonoma">Navegação autônoma com planejamento de trajetória e desvio de obstáculos (particle filter, DWA ).</a>
  - <a href="#como-rodar-tudo-junto">Roda todos pacotes de uma vez.</a>

<hr>

<h3 id="requisitos"> 🧾 Requisitos para rodar o projeto</h3>

- <a target="_blank" href="https://www.ros.org/">ROS</a>
- <a target="_blank" href="http://gazebosim.org/">Gazebo</a>
    ```shell
    sudo apt install ros-noetic-desktop-full
    ```
- <a target="_blank" href="https://www.turtlebot.com/">Turtlebot</a>
    ```shell
    sudo apt install ros-noetic-turtlebot3 && 
    sudo apt install ros-noetic-turtlebot3-gazebo
    ```
- <a target="_blank" href="http://wiki.ros.org/hector_slam">Pacotes de mapeamento (SLAM hector mapping)</a>
    ```shell
    sudo apt install ros-noetic-hector-slam
    ```
- <a target="_blank" href="http://wiki.ros.org/teleop_tools">Pacotes de teleoperação </a>
    ```shell
    apt install ros-noetic-teleop-tools
    ```
- <a target="_blank" href="http://wiki.ros.org/amcl">Pacotes de localização (AMCL)</a>
    ```shell
    sudo apt install ros-noetic-amcl
    ```
- <a target="_blank" href="http://wiki.ros.org/navigation">Pacotes de navegação (Path Planning, DWA)</a>
    ```shell
    sudo apt install ros-noetic-navigation
    ```

<hr>

<h3 id="como-rodar"> 🏗️ Rodar o projeto</h3>

- Sempre que abrir um novo terminal utilize o comando para atualizar a variável de ambiente que armazena o caminho para os pacotes do ROS:
    ```shell
    source ~/catkin_ws/devel/setup.sh
    ```
- Construa os pacotes:
    ```shell
    cd ~/catkin_ws && catkin_make
    ```
- Atualize os pacotes:
    ```shell
    rospack profile
    ```

<h2 id="como-rodar-robo-mundo"> 🌍 Rodar o mundo e inserir o robo dentro dele</h2>

- Rode o comando no terminal:
    ```shell
    roslaunch turtle_bot_trabalho1 turtlebot_simulator_in_world.launch
    ```

<h2 id="como-rodar-mapeamento"> :world_map: Mapeamento (hectormapping)</h2>

- Com o mundo e robo já incializados rode o comando abaixo no terminal e posteriormente controle o robo pelo mouse para que ele vá conhecendo e armazenando o mapa:
    ```shell
    roslaunch turtle_bot_trabalho1 slam_localization.launch.launch slam_mode:=true mouse_teleop:=true
    ```
- Após o mapa ter sido explorado rode o comando abaixo para salvar o mapa para que possa ser carregado posteriormente:
    ```shell
    rosrun map_server map_saver -f `rospack find turtle_bot_trabalho1`/maps/guimaraes_house_world 
    ```  

<h2 id="como-rodar-localizacao"> 🧭 Localização (amcl)</h2>

- Com o mundo e robo já incializados rode o comando abaixo no terminal, ative a visualização do array de posições (PoseArray) no RViz e posteriormente vá andando com o robo utilizando teleoperação pelo mouse para perceber que ele vai eliminando posições a medida que ele vai comparando a leitura do sensor com o mapa:
    ```shell
    roslaunch turtle_bot_trabalho1 slam_localization.launch.launch localization_mode:=true mouse_teleop:=true
    ```  

<h2 id="como-rodar-navegacao-autonoma"> 🔀 Navegação autônoma com planejamento de trajetória e desvio de obstáculos (particle filter, DWA )</h2>

- Com o mundo e robo já incializados rode o comando abaixo no terminal, ative as duas visualizações de trajetárias (Path) no RViz e posteriormente sete um destino (botão "2D Nav Goal") para que o robo trace a trajetória baseada no mapa de custo global e também vá trançando novas trajetórias curtas utilizando o DWA baseado no mapa de custo local para se desviar de novos obstáculos:
    ```shell
    roslaunch turtle_bot_trabalho1 navigation.launch 
    ```  

<h2 id="como-rodar-tudo-junto"> 💯 Roda todos pacotes de uma vez</h2>

- Rode o comando abaixo no terminal para que o ROS execute um arquivo launch que inicializará o Gazebo já com o mundo e o robo, inicializará o RViz, carregará o mapa, inicializará o algoritmo de localização (amcl) e os algoritmos de navegação autônoma de planejamento de trajetória (filtro de partícula com algoritmo estrela) e o DWA:
    ```shell
    roslaunch turtle_bot_trabalho1 main_navigation.launch gazebo_gui:=true
    ```
<hr>