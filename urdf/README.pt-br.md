![Capa](../docs/banner_urdf.png)

# 📏 Robô genérico de VSS

## 📂 Visão geral

Dentro dessa pasta estão os arquivos de descrição para um robô genérico de VSS.

- **generic\_vss\_robot.urdf** - Descrição em [URDF](http://wiki.ros.org/urdf) do robô, gerada com a extensão [SW2URDF](http://wiki.ros.org/sw_urdf_exporter) do SolidWorks
- **generic\_vss\_robot.xacro** - Arquivo principal que importa todos os outros
- **generic\_vss\_robot.gazebo** - Especificações específicas do Gazebo, por exemplo, propriedades físicas e configuração de cores
  - **generic\_vss\_team.gazebo** - Configurações diferentes de cores para os dois times (Amarelo/Azul) e para cada robô (1..3) de cada time
- **motor.xacro** - Configuração do atuador e da transmissão para controle dos motores com a bilioteca [hardware\_interface](http://wiki.ros.org/ros_control#Hardware_Interfaces) de [ros\_control](http://wiki.ros.org/ros_control)
- **generic_vss_robot.pdf** - Desenho técnico detalhado do modelo 3D do robô 

Dentro da pasta **meshes/**, estão os arquivos .stl de cada parte do robô e, dentro de **media/materials/scripts/**, estão os [OGRE scripts](http://wiki.ogre3d.org/Materials) para definir diferentes cores e texturas.

## 📜 Parâmetros principais

|      Parâmetro      |           Valor|Unidade|
|:-------------------:|---------------:|:------|
|    Raio da roda     |              25| mm    |
|  Espessura da roda  |               8| mm    |
| Separação das rodas |              55| mm    |
|        Altura       |              62| mm    |
|     Comprimento     |              78| mm    |
|       Largura       |              78| mm    |
| Densidade das rodas |            1150| kg/m³ |
|  Material das rodas |           Nylon| \-    |
|  Material do corpo  |  50% infill ABS| \-    |
|  Densidade do corpo |             510| kg/m³ |
|     Massa total     |           \~180| g     |
