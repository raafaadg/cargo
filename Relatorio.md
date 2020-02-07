# Relatória da Solução

Como teste proposto, é desejado solucionar um problema do tipo VRPTW.

Este desafio consiste em decidir qual a melhor rota que um determinado veículo deve seguir conforme as orientações fornecidas.

Em nosso caso, foram fornecidos um conjunto de dados sobre 30 locais de entrega, contendo sua Latitude, Longitude, Peso e Volume da Carga e Início e Término da Janela.

Também foram disponibilizadas algumas informações relacionadas aos tres tipos de veículos disponiveis para realizar os transportes.

Inicialmente, realizou-se um tratamento nos dados fornecidos, visando estrutura-los de uma melhor forma para leitura. Na sequência, foi gerada a 
matrix de distancias e para tal, foi utilizado o calculo de haversine para obter a distância entre dois conjustos de Latitude e Longitude.

Para está solução, temos com objetivo encontrar a sequencia de rotas (tendo em vista as restrições de peso e volume dos vaiculos) dado os pontos de entrega
que foram informados, que retorne o menor custo da viagem. Desta forma, estamos diante de um problema de minimização restrito pelos limites de volume e peso 
descritos na questao.

Hoje existem bibliotecas que nos auxíliam na modelagem de problemas de otimização. Para a seguinte solução, optou-se por utilizar a biblioteca
do Google, chamda OR Tools.

Para realizar a modelagem de um problema VRPTW com o OR Tools, devemos realizar as seguintes ferramentas:

### Criar um conjunto das rotas possíves

### Em seguida, é necessário adicionar uma dimensão ao nosso problema. Como estamos trabalhando com tempo, devemos adicionar uma dimensão desse exato formato.
Vale dizer que as distâncias entre latitude e longitude estão sendo interpreta como "Unindade de deslocamento" e podem ser equivalentes ao tempo.

### Deve-se ajustar as janelas de início e término com relação aos pontos de entrega. De um ponto de vista matemático, tanto as janelas quantos os limites
são responsáveis por gerar regras a fim de ordenar a prioridade de entrega, visando aumentar a eficiência de um indicador (valor total das entregas).

### Além da dimensão do tempo, devemos adicionar outras duas dimensões de capacidade, as quais serão responsáveis por realizar o monitoramento do peso e volume de cada veículo.

### Por fim, após toda a correta modelagem, escolheu-se aplicar um método de busca euristica para encontrar o caminho de menor custo.
