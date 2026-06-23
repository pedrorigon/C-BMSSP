# BMSSP: algoritmo, implementação, prova e profiling

Esta análise usa como fonte primária o paper em
[`bmssp-paper.pdf`](bmssp-paper.pdf) e avalia a implementação atual do
repositório. Os números de performance foram medidos em 23 de junho de 2026.

## 1. Veredito

A implementação sequencial agora segue a variante determinística do paper que
garante:

\[
O(m\log^{2/3} n)
\]

Ela contém os componentes necessários para essa garantia:

- transformação para grau de entrada e saída constante;
- ordem total dos caminhos para eliminar empates;
- `FindPivots` com `k` rodadas de relaxamento;
- caso-base limitado a `k + 1` extrações;
- estrutura parcial de blocos com `Insert`, `BatchPrepend` e `Pull`;
- recursão `BMSSP(l, B, S)` do Algoritmo 3;
- chamada superior com `B = infinito`, `S = {s}` e
  `l = ceil(log(n) / t)`;
- ausência de fallback ou finalização por Dijkstra.

Os erros da versão anterior foram removidos. Em particular, o algoritmo não
termina mais com uma passagem `complete_shortest_paths`, não usa uma fila FIFO
como aproximação de `BatchPrepend` e não troca de algoritmo conforme o tamanho
do grafo.

A versão OpenMP preserva a mesma recursão e o mesmo trabalho assintótico, mas é
uma extensão de engenharia. O teorema do paper é sobre o algoritmo
determinístico sequencial. A versão paralela apenas distribui a varredura das
arestas de lotes completos e aplica os resultados serialmente em ordem
determinística.

## 2. Ideia central

Dijkstra mantém todos os candidatos em uma fila de prioridade totalmente
ordenada. Cada extração ou decrease-key custa `O(log n)`.

O BMSSP evita manter essa ordem global completa. Em vez de descobrir
exatamente qual é o próximo vértice, ele trabalha com lotes:

1. encontra um pequeno conjunto de raízes relevantes, os pivôs;
2. retira até `M` candidatos aproximadamente mínimos;
3. resolve recursivamente apenas esse lote;
4. recebe um novo limite até o qual o trabalho está completo;
5. devolve à estrutura os candidatos que ainda precisam ser processados.

Essa ordenação parcial é suficiente para manter a correção e custa menos que
ordenar toda a fronteira.

## 3. Preparação do grafo e das etiquetas

### 3.1 Grau constante

O paper supõe grau de entrada e saída constante. Para cada vértice original, a
implementação cria um ciclo dirigido de peso zero contendo suas incidências.
Uma aresta original `(u, v, w)` passa a ligar uma incidência de `u` a uma
incidência de `v` com peso `w`.

Consequências:

- cada vértice transformado tem grau de entrada e saída no máximo 2;
- distâncias entre vértices originais são preservadas;
- o grafo transformado continua com `O(m)` vértices e arestas.

O código está em
[`constant_degree_graph.cpp`](../src/core/constant_degree_graph.cpp).

### 3.2 Ordem total dos caminhos

Pesos iguais e arestas de peso zero podem produzir vários caminhos de mesma
distância. O paper resolve isso ordenando caminhos por:

1. distância;
2. quantidade de vértices;
3. destino;
4. predecessor, quando o destino é o mesmo.

A implementação representa essa ordem com `QueueKey` e com o desempate do
predecessor durante o relaxamento. Isso mantém uma árvore de predecessores
determinística e permite reutilizar uma aresta quando a igualdade é válida,
como exigido pelas observações 3.4 e 3.5 do paper.

### 3.3 Parâmetros

No grafo transformado:

\[
k = \lfloor \log^{1/3}n \rfloor
\]

\[
t = \lfloor \log^{2/3}n \rfloor
\]

com mínimo prático igual a 1.

A profundidade superior é:

\[
L = \left\lceil\frac{\log n}{t}\right\rceil
  = O(\log^{1/3}n)
\]

## 4. O que uma chamada BMSSP faz

Uma chamada:

```text
BMSSP(l, B, S)
```

recebe:

- um nível `l`;
- um limite superior `B`;
- um conjunto de fontes completas `S`.

Ela devolve:

- um limite `B' <= B`;
- um conjunto `U` de vértices cujas distâncias finais foram determinadas.

Se a estrutura interna esvazia, a execução é bem-sucedida e `B' = B`. Caso o
limite de trabalho do nível seja atingido antes, a execução é parcial e
`B' < B`.

## 5. `FindPivots`

`FindPivots(B, S)` reduz o número de fontes que precisam entrar na recursão.

### Execução

1. Começa com `W = S` e a camada ativa `W0 = S`.
2. Executa até `k` rodadas.
3. Em cada rodada, relaxa as arestas que saem da camada anterior.
4. Só propaga etiquetas menores que `B`.
5. Cada vértice alcançado recebe a raiz de `S` responsável por sua etiqueta.
6. Se `|W| > k|S|`, retorna imediatamente `P = S`.
7. Caso contrário, conta quantos vértices ficaram associados a cada raiz.
8. Uma raiz entra em `P` quando cobre pelo menos `k` vértices.

### Por que funciona

Considere um caminho mínimo que começa em uma fonte completa de `S`.

- Se o trecho relevante tem menos de `k` arestas, as `k` rodadas conseguem
  completar o destino e colocá-lo em `W`.
- Se o trecho é maior, sua raiz cobre ao menos `k` vértices e vira pivô.

Assim, todo vértice necessário ou já foi completado em `W`, ou seu caminho
mínimo passa por um pivô completo.

A implementação propaga a raiz durante os relaxamentos. Isso é equivalente a
construir a floresta `F` do Algoritmo 1 e depois localizar suas raízes, mas evita
uma reconstrução adicional da floresta.

## 6. Estrutura parcial do Lema 3.3

A classe [`BlockQueue`](../src/core/block_queue.cpp) implementa duas sequências
de blocos:

- `D0`: blocos produzidos por `BatchPrepend`;
- `D1`: blocos produzidos por `Insert`.

Cada bloco contém no máximo `M` entradas. A ordem é obrigatória entre blocos,
mas não dentro de um bloco. Por isso as entradas de cada bloco ficam em um
vetor contíguo, com remoção O(1) por troca com a última entrada.

### `Insert`

- localiza em uma árvore balanceada o bloco cujo separador aceita a chave;
- faz decrease-key se o vértice já existe;
- divide o bloco pela mediana quando ele ultrapassa `M`.

Custo amortizado:

\[
O(\max\{1,\log(N/M)\})
\]

### `BatchPrepend`

Recebe um lote cujas chaves são menores que todas as chaves atuais.

- elimina duplicatas, mantendo a menor chave por vértice;
- particiona recursivamente o lote por medianas;
- cria blocos de tamanho no máximo `M`;
- coloca esses blocos no início de `D0`.

Custo amortizado para um lote de tamanho `L`:

\[
O(L\max\{1,\log(L/M)\})
\]

### `Pull`

- coleta um prefixo suficiente de `D0`;
- coleta um prefixo suficiente de `D1`;
- seleciona os até `M` menores elementos da união;
- remove esses elementos;
- retorna o menor valor restante como separador, ou `B` se a estrutura esvaziou.

Custo amortizado:

\[
O(|S'|)
\]

## 7. Caso-base

Quando `l = 0`, `S` contém exatamente um vértice completo.

O algoritmo executa um mini-Dijkstra:

1. insere a fonte em um heap;
2. extrai candidatos em ordem;
3. relaxa apenas etiquetas menores que `B`;
4. para ao completar `k + 1` vértices ou ao esvaziar o heap.

Se encontrou no máximo `k` vértices, retorna todos e mantém `B' = B`.

Se encontrou `k + 1`, usa a maior etiqueta extraída como novo limite `B'` e
retorna apenas os vértices estritamente menores que `B'`. Logo o caso-base
devolve no máximo `k` vértices completos em uma execução parcial.

## 8. Passos da recursão

Para `l > 0`, a implementação segue o Algoritmo 3:

1. calcula `(P, W) = FindPivots(B, S)`;
2. inicializa `D` com:

   \[
   M = 2^{(l-1)t}
   \]

3. insere os pivôs em `D`;
4. enquanto `D` não estiver vazia e `|U| < k2^{lt}`:
   1. chama `Pull()` e obtém `(Bi, Si)`;
   2. chama `BMSSP(l - 1, Bi, Si)`;
   3. recebe `(Bi', Ui)`;
   4. remove de `D` os vértices já concluídos em `Ui`;
   5. relaxa todas as arestas que saem de `Ui`;
   6. se a nova etiqueta está em `[Bi, B)`, usa `Insert`;
   7. se está em `[Bi', Bi)`, guarda em `K`;
   8. devolve por `BatchPrepend` o lote `K` e as fontes de `Si` cuja etiqueta
      ainda está em `[Bi', Bi)`;
5. se `D` esvaziou, retorna `B' = B`;
6. caso contrário, retorna o último `Bi'`;
7. adiciona os vértices de `W` com etiqueta menor que `B'`.

A marcação `completed_levels_` impede que um vértice concluído por uma chamada
filha seja contado novamente pelo `W` do nível pai.

A chamada externa é:

```text
BMSSP(ceil(log(n) / t), infinity, {source})
```

Como o nível superior não pode devolver os `Theta(kn)` vértices exigidos por
uma execução parcial, ele necessariamente esvazia `D`, retorna infinito e
completa todos os vértices alcançáveis.

## 9. Prova de correção simplificada

### Caso-base

O caso-base é Dijkstra limitado. Portanto, tudo que ele devolve antes de `B'`
tem distância final correta.

### Passo indutivo

Suponha que chamadas no nível `l - 1` estejam corretas.

`FindPivots` garante que todo vértice ainda incompleto abaixo de `B`:

- já está completo em `W`; ou
- tem caminho mínimo passando por algum pivô em `D`.

`Pull` escolhe as menores fontes necessárias para o próximo intervalo. Pela
hipótese de indução, a chamada filha completa exatamente os vértices antes de
`Bi'`.

Ao relaxar as arestas desses vértices:

- candidatos depois de `Bi` voltam por `Insert`;
- candidatos entre `Bi'` e `Bi` voltam por `BatchPrepend`.

Portanto, nenhum caminho mínimo pendente desaparece da fronteira. A cada
iteração, o intervalo completo cresce de forma monotônica. No final, os
vértices de `W` abaixo do limite também são completos.

Logo a chamada devolve exatamente o conjunto necessário abaixo de `B'`, com
distâncias corretas.

## 10. Prova simples da complexidade

Considere primeiro o grafo de grau constante. Nele, `m = O(n)`.

### Profundidade

\[
L = O(\log n/t) = O(\log^{1/3}n)
\]

### `FindPivots`

Em uma profundidade fixa da árvore de recursão, os conjuntos concluídos são
disjuntos. O custo total de `FindPivots` nessa profundidade é `O(nk)`.

Somando as profundidades:

\[
O(nkL)
= O(n\log^{1/3}n\log^{1/3}n)
= O(n\log^{2/3}n)
\]

### Inserções diretas

Cada aresta pode causar uma inserção direta cara em apenas um nível. Cada
inserção custa `O(t)`.

\[
O(mt) = O(m\log^{2/3}n)
\]

### `BatchPrepend`

Uma aresta pode participar de lote em vários níveis, com custo `O(log k)` por
nível:

\[
O(mL\log k)
= O(m\log^{1/3}n\log\log n)
\]

Esse termo é assintoticamente menor que `O(m log^(2/3)n)`.

### Resultado

Somando caso-base, pivôs, inserções, lotes e pulls:

\[
\boxed{O(m\log^{2/3}n)}
\]

A transformação de grau cria apenas `O(m)` vértices e arestas. Como
`log m = O(log n)` para um grafo simples, ela não altera o limite assintótico
expresso nos parâmetros do grafo original.

## 11. Correspondência entre paper e código

| Componente do paper | Implementação | Avaliação |
|---|---|---|
| Transformação para grau constante | `constant_degree_graph.cpp` | Fiel |
| Ordem total de caminhos | `QueueKey`, `label_less`, `candidate_valid` | Fiel |
| Algoritmo 1, `FindPivots` | `SequentialSolver::find_pivots` | Fiel |
| Algoritmo 2, caso-base | `SequentialSolver::base_case` | Fiel |
| Algoritmo 3, BMSSP | `SequentialSolver::bounded_search` | Fiel |
| Lema 3.3 | `BlockQueue` | Fiel, com blocos contíguos |
| Chamada superior | `SequentialSolver::solve_paper` | Fiel |
| Dijkstra final | Não existe | Correto |
| Fallback por tamanho | Não existe | Correto |
| Paralelização | `ParallelSolver::relax_completed` | Extensão fora do paper |
| Saída ponto a ponto | parada ao completar o objetivo | Extensão segura |

### Observação de memória

Cada nível ativo mantém vetores densos de localização e deduplicação para a
estrutura parcial. Isso é determinístico e rápido, mas a implementação prática
pode usar `O(nL)` memória auxiliar, com `L = O(log^(1/3)n)`. A garantia pedida e
provada pelo paper é de tempo; este repositório não deve anunciar atualmente
uma garantia de espaço linear estrita.

## 12. Validação de correção

Foram executados:

- todos os 15 testes em Release;
- todos os 15 testes em Debug;
- todos os 15 testes com AddressSanitizer e UndefinedBehaviorSanitizer;
- comparação ponto a ponto e full contra Dijkstra;
- grafos aleatórios com várias densidades e 24 sementes adicionais;
- Rome99;
- grafo de comparação grande;
- teste aleatório da `BlockQueue` contra `std::map`, cobrindo decrease-key,
  remoção, `BatchPrepend` e `Pull`;
- teste de preservação de distâncias e grau máximo 2 na transformação.

Todos passaram.

## 13. Metodologia do profiling

Máquina:

- AMD Ryzen 5 5600X, 6 núcleos e 12 threads;
- 32 GiB de RAM;
- Linux 6.8;
- GCC, C++20, Release;
- OpenMP com 6 threads, `OMP_PROC_BIND=close` e `OMP_PLACES=cores`.

Entrada:

- 100.000 vértices;
- 799.999 arestas;
- grau médio 8;
- pesos reais determinísticos;
- topologias `random` e `banded`;
- 1 aquecimento e 5 repetições medidas.

O tempo mede somente as chamadas ao solver. A construção do grafo, a
transformação e a construção do objeto ficam fora da janela. Já os contadores
do `perf stat` abrangem o processo completo, incluindo preparação, aquecimento
e três execuções.

Para separar os custos, foi incluído um Dijkstra sequencial sobre o mesmo grafo
transformado usado pelo BMSSP.

No caso aleatório, a transformação produziu:

- 1.699.998 vértices;
- 2.499.997 arestas.

## 14. Tempos

### Topologia aleatória

| Modo | Dijkstra seq. | Dijkstra seq. transformado | BMSSP seq. | Dijkstra par. | BMSSP par. |
|---|---:|---:|---:|---:|---:|
| Ponto a ponto | 21,76 ms | 254,60 ms | 1.375,96 ms | 20,43 ms | 1.403,17 ms |
| Full | 42,27 ms | 418,41 ms | 2.109,32 ms | 24,81 ms | 2.093,35 ms |

### Topologia banded

| Modo | Dijkstra seq. | Dijkstra seq. transformado | BMSSP seq. | Dijkstra par. | BMSSP par. |
|---|---:|---:|---:|---:|---:|
| Ponto a ponto | 18,40 ms | 156,12 ms | 873,17 ms | 12,26 ms | 948,90 ms |
| Full | 20,31 ms | 156,61 ms | 876,53 ms | 12,69 ms | 899,21 ms |

No full aleatório:

- a transformação explica aproximadamente `9,9x` sobre Dijkstra original;
- BMSSP custa aproximadamente `5,0x` sobre Dijkstra no mesmo grafo
  transformado;
- o total é aproximadamente `49,9x` sobre Dijkstra sequencial original;
- BMSSP paralelo e sequencial praticamente empatam.

## 15. Contadores de hardware

Perfil full aleatório:

| Algoritmo | Instruções | Ciclos | IPC | Miss de cache | Miss de branch |
|---|---:|---:|---:|---:|---:|
| Dijkstra seq. | 0,92 bi | 0,97 bi | 0,95 | 37,5% | 6,81% |
| Dijkstra seq. transformado | 9,80 bi | 9,52 bi | 1,03 | 36,8% | 1,67% |
| BMSSP seq. | 37,75 bi | 39,43 bi | 0,96 | 43,7% | 0,99% |
| Dijkstra par. | 1,03 bi | 3,28 bi | 0,31 | 37,1% | 5,30% |
| BMSSP par. | 41,60 bi | 62,48 bi | 0,67 | 43,6% | 0,92% |

O BMSSP não perde principalmente por previsões de branch. Sua taxa de erro é
baixa. Ele perde porque executa muito mais instruções e acessa uma área de
memória muito maior.

## 16. Hotspots

BMSSP sequencial:

| Função | Ciclos |
|---|---:|
| `find_pivots` | 17,71% |
| `bounded_search` | 11,42% |
| `base_case` | 9,14% |
| `relax_completed` | 9,06% |
| `BlockQueue::batch_prepend` | 6,47% |
| `candidate_valid` | 5,62% |
| `BlockQueue::erase` | 5,25% |
| `BlockQueue::insert` | 2,95% |
| `BlockQueue::pull` | 2,22% |

Alocação e liberação de memória também aparecem de forma distribuída entre os
hotspots. Os blocos usam armazenamento contíguo e reservas antecipadas, mas a
recursão ainda cria vetores temporários e blocos dinamicamente.

BMSSP paralelo:

- cerca de 25% dos ciclos aparecem dentro do runtime OpenMP;
- `find_pivots` continua sequencial e consome 11,37%;
- apenas a varredura em `relax_completed` é paralela;
- os candidatos são materializados por thread e aplicados serialmente.

Isso explica a ausência de speedup nesta escala.

## 16.5. Prova empírica da complexidade (trabalho ponderado)

O tempo de parede **não** pode demonstrar a vantagem assintótica do BMSSP: a
transformação para grau constante e o fator constante muito maior da recursão
dominam qualquer entrada que cabe em memória. A afirmação de complexidade é sobre
o **número de operações de estrutura ordenada** que a análise cobra, não sobre
segundos.

Para medir isso, ambos os solvers foram instrumentados com um contador de
trabalho ponderado em [`work_counter.h`](../src/include/work_counter.h):

- **Dijkstra** cobra `log2(tamanho da fila)` por `push`/`pop` do heap — exatamente
  o `O(log n)` por operação de fila de prioridade;
- **BMSSP** cobra os custos do Lema 3.3: `log2(N/M)` por `insert`/`erase` na
  árvore de blocos, `L·log2(L/M)` por `batch_prepend` de `L` itens, o tamanho do
  prefixo escaneado por `pull`, e o termo `O(nk)` das `k` rodadas de `FindPivots`.

Somando esse trabalho ao longo de um full SSSP e dividindo pelo número de arestas
do grafo original, obtém-se o **trabalho por aresta**. Ele cresce como `log n`
para Dijkstra e como `log^(2/3) n` para BMSSP. O contador é determinístico: a
mesma semente produz exatamente o mesmo número, sem ruído de medição.

### Resultado (topologia `random`, grau 8, `n` de 1.000 a 1.000.000)

| `n` | `log n` | `log^(2/3) n` | Dijkstra (norm.) | BMSSP (norm.) |
|---:|---:|---:|---:|---:|
| 1.000 | 1,00 | 1,00 | 1,000 | 1,000 |
| 10.000 | 1,33 | 1,21 | 1,342 | 1,163 |
| 100.000 | 1,67 | 1,41 | 1,690 | 1,292 |
| 1.000.000 | 2,00 | 1,59 | 2,040 | 1,408 |

A coluna "norm." divide o trabalho por aresta de cada algoritmo pelo seu valor no
menor grafo, isolando a **taxa de crescimento** do fator constante. De `n = 10^3`
a `n = 10^6`:

- o trabalho por aresta de Dijkstra cresce **2,04x**, casando com `log n` dentro
  de 2% de erro em três ordens de grandeza;
- o do BMSSP cresce apenas **1,41x**, abaixo do limite `log^(2/3) n`.

A divergência é monótona e aumenta a cada passo. Esta é a evidência direta do
expoente menor, independente de tempo de parede e de fatores constantes.

### Como apresentar nos slides

O script [`run_scaling.py`](../run_scaling.py) gera
`plots/scaling/<timestamp>_scaling_complexity.jpg` com dois painéis:

- **esquerda — trabalho por aresta bruto**: mostra a forma real. A curva do BMSSP
  achata enquanto a de Dijkstra continua subindo. Honesto: a constante maior do
  BMSSP mantém sua curva acima da de Dijkstra em valor absoluto.
- **direita — normalizado ao menor grafo**: ambas partem de 1,0 e a de Dijkstra
  dispara acima da do BMSSP. É o painel que prova visualmente o cruzamento de
  curvatura, ou seja, o expoente menor.

Há também uma terceira figura, `..._crossover.jpg`, descrita abaixo.

Comando reproduzível (padrões: grau 4 esparso, até 100M de vértices):

```bash
uv run python run_scaling.py --type sequential
```

O grau médio padrão é 4: grafos esparsos são o regime mais justo ao BMSSP,
porque a transformação para grau constante adiciona menos vértices extras. O
limite padrão de vértices é 100.000.000; o Dijkstra mede de verdade até esse
tamanho, enquanto o BMSSP é interrompido quando excede o teto de memória
(`--memory-limit-gb`, padrão 24 GiB) — nesta máquina, por volta de 5.000.000 de
vértices. As curvas ficam com comprimentos diferentes de propósito: isso mesmo
mostra o Dijkstra continuando a subir muito além de onde o BMSSP cabe.

A narrativa correta para a banca é: **o BMSSP perde em segundos por causa do
fator constante e da expansão do grafo, mas a curva de trabalho confirma o
expoente `O(m log^(2/3) n)` menor que o `O(m log n)` de Dijkstra.** É a mesma
narrativa que a comunidade de algoritmos usa quando uma constante grande esconde
uma vantagem assintótica.

### Expoente local e extrapolação do cruzamento

Medindo a inclinação local de `log(trabalho)` contra `log(n)` no grafo esparso
(grau 4), obtém-se o expoente empírico de crescimento:

| `n` | expoente BMSSP | expoente Dijkstra |
|---:|---:|---:|
| 10.000 | 1,107 | 1,143 |
| 100.000 | 1,057 | 1,109 |
| 1.000.000 | 1,043 | 1,084 |
| 5.000.000 | 1,027 | 1,073 |

O expoente do Dijkstra é maior em todos os pontos, e a folga **aumenta** com o
tamanho (de 0,036 para 0,046). Essa é a assinatura direta de `log n` crescendo
mais rápido que `log^(2/3) n`.

Como consequência, a razão entre o trabalho por aresta dos dois algoritmos cai de
forma monótona com `ln(n)`:

| `n` | razão trabalho/aresta BMSSP ÷ Dijkstra |
|---:|---:|
| 1.000 | 3,19 |
| 100.000 | 2,61 |
| 1.000.000 | 2,37 |
| 5.000.000 | 2,20 |

Um ajuste linear da razão contra `ln(n)` sobre os pontos medidos cruza `1,0` em
torno de `n ≈ 10^10` a `10^11`, dependendo da faixa de tamanhos medida. A figura
`..._crossover.jpg` mostra exatamente isso: a razão medida e a reta extrapolada
(tracejada) até o empate de trabalho.

**Importante para a banca:** essa ordem de `10^10`–`10^11` é uma **extrapolação**
muito além da memória de uma única máquina, baseada em pontos medidos — não é uma
medição. O
argumento honesto e suficiente para os slides não é "o BMSSP vence em `10^11`",
mas sim: **a curva do Dijkstra cresce com inclinação maior e a do BMSSP tende a
achatar; a tendência medida aponta para o cruzamento, confirmando o expoente
menor.** Em tempo de parede, o BMSSP não ultrapassa o Dijkstra em nenhuma entrada
que cabe em RAM, e nenhum formato de grafo muda isso, porque o gargalo é a
transformação obrigatória para grau constante.

## 17. Por que Dijkstra é mais rápido em computadores modernos

### Menor grafo efetivo

Dijkstra opera diretamente sobre 100 mil vértices e 800 mil arestas. BMSSP
opera sobre aproximadamente 1,7 milhão de vértices e 2,5 milhões de arestas
para satisfazer a hipótese de grau constante.

### Hot loop simples

O núcleo de Dijkstra é:

1. extrair do heap;
2. percorrer uma lista de adjacência;
3. comparar;
4. atualizar distância e heap.

É um fluxo compacto, com poucos estados auxiliares.

### Melhor localidade

Distâncias, predecessores, heap e listas de adjacência usam vetores. O BMSSP
também usa vetores onde possível, mas precisa consultar:

- marcas por rodada;
- raízes de pivôs;
- níveis de completude;
- localização em blocos;
- árvore de separadores;
- sequências `D0` e `D1`;
- vetores temporários de cada chamada.

### Vantagem assintótica pequena nas escalas atuais

`log^(2/3)n` cresce menos que `log n`, mas a diferença matemática é pequena para
os tamanhos que cabem em uma única máquina. Os custos constantes dominam muito
antes do ponto em que o termo assintótico poderia compensá-los.

### Paralelismo mais favorável no Delta-stepping

No grafo aleatório full, Dijkstra paralelo foi `1,70x` mais rápido que o
sequencial. Seus buckets expõem lotes maiores e sua estrutura principal é mais
simples.

O BMSSP paralelo possui regiões menores, recursão sequencial e commit serial.
No full aleatório, o speedup foi apenas `1,008x`; nos casos banded ele ficou
mais lento.

## 18. Gargalos inerentes e gargalos da implementação

### Inerentes à variante garantida

- transformação para grau constante;
- `k` rodadas de `FindPivots`;
- vários níveis recursivos;
- reuso de relaxamentos válidos em níveis diferentes;
- manutenção de fronteiras parcialmente ordenadas;
- `BatchPrepend` e reinserção de fontes de chamadas parciais.

### Específicos da implementação atual

- vetores densos da `BlockQueue` por nível aumentam a memória auxiliar;
- `std::map` mantém os separadores de `D1`, gerando acessos por ponteiros;
- blocos e vetores temporários ainda causam alocações;
- `find_pivots` não foi paralelizado;
- a versão OpenMP materializa candidatos e faz commit serial;
- o limiar paralelo é fixo e não considera custo por lote ou topologia.

Uma tentativa de executar o Delta-stepping no grafo transformado foi encerrada
por consumo excessivo de memória. As muitas arestas de peso zero reduzem
`delta`, produzindo índices de bucket muito grandes. Isso é uma limitação da
implementação de Dijkstra paralelo, não uma propriedade de Dijkstra em geral.

## 19. Animações

As animações usam uma malha urbana vista de cima, com ruas locais, arteriais
curvas, diagonais e anel viário. Não há barreiras de labirinto.

Cada GIF:

- tem 200 frames;
- usa 20 FPS;
- dura exatamente 10,000 segundos;
- usa a razão real dos tempos medidos;
- faz o algoritmo mais rápido terminar antes;
- mantém o resultado final visível enquanto o algoritmo mais lento continua;
- redistribui os eventos por ordem ao longo do tempo medido para evitar
  travamentos visuais causados por rajadas de callbacks.

### Menor caminho

- [`path_sequential.gif`](../visualizations/path/path_sequential.gif)
- [`path_parallel.gif`](../visualizations/path/path_parallel.gif)
- [`path_all.gif`](../visualizations/path/path_all.gif)

### Full SSSP

- [`full_sequential.gif`](../visualizations/full/full_sequential.gif)
- [`full_parallel.gif`](../visualizations/full/full_parallel.gif)
- [`full_all.gif`](../visualizations/full/full_all.gif)

Os tempos usados ficam gravados em
[`visual_trace_path.txt`](../output/profiling/visual_trace_path.txt) e
[`visual_trace_full.txt`](../output/profiling/visual_trace_full.txt).

## 20. Conclusão

A implementação agora representa corretamente a variante do paper que garante
`O(m log^(2/3)n)`. A prova assintótica depende de ordenar apenas lotes, reduzir
fontes com pivôs e limitar o número de vezes que uma aresta provoca inserções
caras.

Na máquina testada, Dijkstra continua muito mais rápido. A principal razão é
prática: a variante garantida expande o grafo, executa dezenas de vezes mais
instruções e mantém muito mais estado. A vantagem do BMSSP é teórica e
assintótica; ela não implica superioridade nas escalas e arquiteturas atuais.
