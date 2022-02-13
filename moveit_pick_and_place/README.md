<h1 align="center" ><b style="color: #03A9F5;">PROJETO FINAL - "Pick and Place" usando MoveIt e o braço robótico Panda</b> </h1>

<h3> :loudspeaker: O que é o projeto? </h3>
<p>O projeto tem como objetivo utilizar as funcionalidades de manipulação de braços robóticos fornecidos pelo MoveIt para manipular um modelo de branço robótico chamado Panda (braço robótico padrão utilizado nos tutoriais do MoveIt) de forma que ele pegue um objeto em uma posição e coloque-o em outra. Este é um projeto da disciplina de <b>robótica aplicada</b> do CEFET-MG campus Leopoldina ministrada pelo professor Vinícius Schettino.</p>

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
- <a href="#como-construir">Construindo o projeto.</a>
- <a href="#como-rodar">Rodando o projeto.</a>

<hr>

<h3 id="requisitos"> 🧾 Requisitos para rodar o projeto</h3>

- <a target="_blank" href="https://www.ros.org/">ROS</a>
- <a target="_blank" href="http://gazebosim.org/">Gazebo (Opcional)</a>
    ```shell
    sudo apt install ros-noetic-desktop-full
    ```
- <p>Atualizando pacotes do ROS </p>
    ```shell
    rosdep update
    sudo apt update
    sudo apt dist-upgrade
    ```
- <a target="_blank" href="http://wiki.ros.org/catkin">Instala o <b>catkin</b> que é o sistema de build do ROS </a>
    ```shell
    sudo apt install ros-noetic-catkin python3-catkin-tools python3-osrf-pycommon
    ```
- <a target="_blank" href="http://wiki.ros.org/wstool">Instala o <b>wstool</b> que são ferramentas de linha de comando para manter um espaço de trabalho de projetos de vários sistemas de controle de versão.</a>
    ```shell
    sudo apt install python3-wstool
    ```
- <a target="_blank" href="http://wiki.ros.org/navigation">Cria um Catkin workspace e baixa o código fonte do MoveIt</a>
    ```shell
    mkdir -p ~/ws_moveit/src
    cd ~/ws_moveit/src

    wstool init .
    wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
    wstool remove  moveit_tutorials  # this is cloned in the next section
    wstool update -t .
    ```

- <p>Instala os pacotes de configuração do modelo do braço robótico Panda</p>
    
    ```shell
    git clone https://github.com/ros-planning/panda_moveit_config.git # bug in binary version
    ```

- <p>O seguinte instalará do Debian quaisquer dependências de pacotes que ainda não estejam em seu espaço de trabalho</p>
    
    ```shell
    cd ~/ws_moveit/src
    rosdep install -y --from-paths . --ignore-src --rosdistro noetic
    ```
- <p>Configure e faça o build dos pacotes no workspace com catkin</p>
    
    ```shell
    cd ~/ws_moveit
    catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
    catkin build
    ```
- <p>Pronto! Tudo configurado. Agora sempre que abrir um novo terminal execute o comando abaixo para atualizar os caminhos do ROS nas variaveis de ambiente</p>
    
    ```shell
    source ~/ws_moveit/devel/setup.bash
    # se for ZSH use o comando abaixo
    source ~/ws_moveit/devel/setup.sh
    ```
<hr>

<h3 id="como-construir"> 🏗️ Construir o projeto</h3>

- Clone o repositorio atual:
    ```shell
    cd ~/catkin_ws/src
    git clone https://github.com/lucaslgr/robotics-ros-gazebo.git
    ```
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

<h3 id="como-rodar"> 🔥 Rodar o projeto</h3>

- Rode o comando no terminal para abrir o RViz e carregar o modelo do braço robótico Panda:
    ```shell
    roslaunch panda_moveit_config demo.launch
    ```
- Rode o pacote que carregara os objetos de superfície e o objeto que será descolado pelo braço. Posteriormente o nó carregado por esse pacote irá fazer com que o braço execute as trajetórias necessárias para pegar, deslocar e colocar o objeto em outro lugar:
    ```shell
    cd ~/catkin_ws/src/moveit_pick_and_place/nodes/

    chmod +x pick_and_place.py

    ./pick_and_place.py
    ```

