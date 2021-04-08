![Capa](../docs/banner_urdf.png)

# 📏 Robô genérico de VSS

## 📂 Visão geral

Dentro dessa pasta estão os arquivos de descrição para um robô genérico de VSS.

- **generic\_vss\_robot.urdf** - Descrição em [URDF](http://wiki.ros.org/urdf) do robô, gerada com a extensão [SW2URDF](http://wiki.ros.org/sw_urdf_exporter) do SolidWorks
- **generic\_vss\_robot.xacro** - Arquivo principal que importa todos os outros
- **generic\_vss\_robot.gazebo** - Especificações específicas do Gazebo, por exemplo, propriedades físicas e configuração de cores
  - **generic\_vss\_team.gazebo** - Configurações diferentes de cores para os dois times (Amarelo/Azul) e para cada robô (0..2) de cada time
- **motor.xacro** - Configuração do atuador e da transmissão para controle dos motores com a biblioteca [hardware\_interface](http://wiki.ros.org/ros_control#Hardware_Interfaces) de [ros\_control](http://wiki.ros.org/ros_control)
- **generic_vss_robot.pdf** - Desenho técnico detalhado do modelo 3D do robô

Dentro da pasta **meshes/**, estão os arquivos .stl de cada parte do robô e, dentro de **media/materials/scripts/**, estão os [OGRE scripts](http://wiki.ogre3d.org/Materials) para definir diferentes cores e texturas.

## 📜 Parâmetros principais

|          Parâmetro           |          Valor | Unidade |
| :--------------------------: | -------------: | :------ |
|         Raio da roda         |             25 | mm      |
|      Espessura da roda       |              8 | mm      |
|     Separação das rodas      |             55 | mm      |
|     Densidade das rodas      |           1150 | kg/m³   |
|    Massa das rodas (cada)    |             18 | g       |
|      Material das rodas      |          Nylon | \-      |
|      Material do corpo       | 50% infill ABS | \-      |
|      Densidade do corpo      |            510 | kg/m³   |
|         Altura total         |             62 | mm      |
|      Profundidade total      |             78 | mm      |
|        Largura total         |             78 | mm      |
|         Massa total          |            180 | g       |
| Momento de inércia total Izz |          0.113 | g m²    |

## 🟣 Parâmetros dos motores

O motor do modelo é inspirado no [Micro Motor de Engrenagens 50:1 da Pololu](https://www.pololu.com/product/3073) de modo a obtermos valores realistas

|           Parâmetro            | Valor | Unidade |
| :----------------------------: | ----: | :------ |
|        Torque máx motor        |    73 | mN m    |
| Aceleração linear máx do robô  |    16 | m/s²    |
| Aceleração angular máx do robô |  1420 | rad/s²  |
|      Velocidade máx motor      |   650 | RPM     |
|   Velocidade linear máx robô   |   1.7 | m/s     |
| Velocidade angular máx do robô |   9.8 | rad/s   |

Os valores foram estimados a partir das relações obtidas desprezando-se o momento de inércia das rodas no eixo y (eixo de rotação do motor)

<img src="https://render.githubusercontent.com/render/math?math=\ddot{x} m=T/R_{roda}">
<img src="https://render.githubusercontent.com/render/math?math=\dot{\omega} I_{zz}=T S_{rodas}/2 R_{roda}">

Onde "S rodas" é a separação entre as duas rodas
