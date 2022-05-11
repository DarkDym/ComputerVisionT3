# ComputerVisionT3
Repositório para o trabalho final da cadeira de Visão Computacional

Para executar o repositório é necessário possuir os pacotes ROS e Husky.

Primeiramente deve-se iniciar a simulação do Husky e depois executar o script de análise do movimento.

Para rodar a simulação do Husky é necessário executar o seguinte comando:
```
roslaunch computer_vision_t3 multi_husky_call.launch world_name:=warehouse_cells navigating:=true
```

Para rodar o script de análise é necessário executar o seguinte comando:
```
python3 /scripts/read_tags.py
```

O relatório final pode ser encontrado [aqui](Relatorio_CMP_197_Final.pdf)