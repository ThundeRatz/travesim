# 🛄 ROS bagfiles

Nessa pasta devem ser salvas as rosbags geradas pela simulação

## Instruções

Uma ferramenta especialmente útil no desenvolvimento de projetos ROS é a criação de bagfiles com auxílio do [rosbag](http://wiki.ros.org/rosbag)

Para gerar uma bagfile a partir da simulação sendo executada, basta executar o comando

```sh
rosbag record -a
```

Todos os tópicos sendo publicados no momento serão salvos no registo da bagfile. É possível selecionar tópicos específicos a serem guardados

```sh
rosbag record -O subset /topic1 /topic2
```

Para obter maiores informações sobre uma rosbag, utilize o comando

```sh
rosbag info <filename>
```

Mais detalhes de como utilizar rosbags [aqui](http://wiki.ros.org/rosbag/Tutorials/Recording%20and%20playing%20back%20data)
