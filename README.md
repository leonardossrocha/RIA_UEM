# Disciplina: Robótica Inteligente e Autônoma 
### Universidade Estadual de Maringá
Projeto da disciplina de Robótica Inteligente e Autônoma para mapeamento e navegação.
<br>
Projeto utilizando o simulador [CoppeliaSim](https://www.coppeliarobotics.com/)
<br>
para habilitar o RemoteAPI no Coppelia utilizando a linguagem python, é necessário que esteja no diretório do projeto (código em python e cena do simulador) os seguintes arquivos:
<br><br>
remoteApi.so <br>
sim.py	<br>
simConst.py <br>
<br>
[Fonte](https://www.coppeliarobotics.com/helpFiles/en/remoteApiClientSide.htm)
<br><br>

## Arquivos do projeto
### Cena para CoppeliaSim

A cena criada para utilização no **simulador CoppeliaSim** está disponível com o nome:

**scene2.ttt**

O projeto de cena criado no CoppeliaSim foi cedido pelo professor da disciplina, Rodrigo Calvo e não sofreu qualquer alteração a não ser aquela necessária para que o simulador se comunique com a IDE escolhida, conforme orientação disponível adiante.
<br><br>

### Código Python

Este trabalho conta com código em Python disponibilizado pelo arquivo:
<br>
**avoidObstacle.py**

### Plotagem

Foi construído um diretório **Plotagens** onde é possível identificar duas plotagens de coleta de informações do robô sobre o ambiente mapeado.


## Informações relevantes para o trabalho

Uma das informações necessárias para a realização do trabalho, diz respeito ao tamanho dos blocos de chão contidos numa cena criada no CoppeliaSim. Essa informação pode ser obtida no fórum do simulador, pelo tópico criado [About floor Size](https://forum.coppeliarobotics.com/viewtopic.php?f=9&t=9505)

## Ferramentas utilizadas

Ferramentas adicionais utilizadas, além das já mencionadas até o momento, é o plugin [gridMap](https://github.com/roboticafacil/coppeliasim_gridmap) mas apresentou problemas em sua estrutura e até o momento da entrega desse relatório, não havia sido solucionado pelo upstream. 

